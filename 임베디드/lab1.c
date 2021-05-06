#include "includes.h"
#define F_CPU	16000000UL	// CPU frequency = 16 Mhz
#include <avr/io.h>	
#include <util/delay.h>
#include <avr/interrupt.h>

#define TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE      
#define ATS75_ADDR 0x98 // 0b10011000, 7비트를 1비트 left shift
#define ATS75_CONFIG_REG 1
#define ATS75_TEMP_REG 0
#define F_SCK 40000UL // SCK 클록 값 = 40 Khz

#define BIT(x) (1 << x)
#define BUZZERTASK_STK_SIZE 512

// 각 태스크의 우선순위 정의
#define BUZZ_PRIO	1
#define WATCH_PRIO	2
#define LED_PRIO	3
#define TEMP_PRIO	4
#define DISP_PRIO	5

// 상태 정의
#define ON 0
#define OFF 1
#define PAUSE 2

//음계명 
#define DO 0 
#define RE 1 
#define MI 2 
#define FA 3 
#define SOL 4 
#define RA 5 
#define SI 6 
#define DDO 7 
#define RRE 8 
#define MMI 9 
#define FFA 10 
#define SSOL 11 
#define PS 20
#define EOS -1

/*
태스크 스택 : 5개 태스크에 대한 스택구조체이다. Buzzer 태스크는 악보 때문에 크게 잡아놓았다.
*/
OS_STK	LedTaskStk[TASK_STK_SIZE];
OS_STK	TempTaskStk[TASK_STK_SIZE];
OS_STK  DispFndTaskStk[TASK_STK_SIZE];
OS_STK  WatchTaskStk[TASK_STK_SIZE];
OS_STK  BuzzerTaskStk[512];

/*
프로그램에 사용되는 모든 동기화 개체를 선언함.
2개의 세마포어, 1개의 메일박스, 1개의 이벤트 그룹을 사용하였다.
*/
OS_EVENT     *FndValSem;
OS_EVENT     *ToneSem;
OS_EVENT	 *TempMbox;
OS_FLAG_GRP  *e_grp;


// FND 디지트 정보와 PORTG를 위한 Selection bit 
unsigned char digit[12] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x27, 0x7f, 0x6f, 0x40, 0x00 };
unsigned char fnd_sel[4] = { 0x01, 0x02, 0x04, 0x08 };
/*
   FND에 표시될 값에 대한 전역변수
   DispFnd 태스크는 무한 루프를 돌면서 아래 값을 FND에 출력한다.
   FndDigit는 소수점을 찍을 자리에 대한 비트값을 저장한다. 
   0x01 = 맨 우측 FND에 소수점 표시, 0x0F = 모든 자리에 소수점 표시
*/
int FndValue;
unsigned char FndDigit;

//각 음계의 타이머 오버플로우 인터럽트에 대한 초기 타이머 값이다.
char f_table[12] = {17, 43, 66, 77, 97, 114, 117, 137, 150, 161, 167, 176 };
// 알람음악에 대한 악보이다. {계명, 박자}의 2차원 배열로 구성하였다. 
int song[57][2] = {
	{ SSOL, 4 }, { MMI, 2 }, { FFA, 2 }, { SSOL, 4 }, { MMI, 2 }, { FFA, 2 }, { SSOL, 2 },
	{ SI, 2 }, { RA, 2 }, { SI, 2 }, { DDO, 2 }, { RRE, 2 }, { MMI, 2 }, { FFA, 2 },
	{ MMI, 4 }, { DDO, 2 }, { RRE, 2 }, { MMI, 4 }, { MI, 4 }, { FA, 2 }, { SOL, 2 }, { RA, 2 },
	{ SOL, 2 }, { FA, 2 }, { SOL, 2 }, { DDO, 2 }, { SI, 2 }, { DDO, 2 },

	{ RA, 4 }, { DDO, 2 }, { SI, 2 }, { RA, 4 }, { SOL, 2 }, { FA, 2 }, { SOL, 2 }, { FA, 2 },
	{ MI, 2 }, { FA, 2 }, { SOL, 2 }, { RA, 2 }, { SI, 2 }, { DDO, 2 }, { RA, 4 },
	{ DDO, 2 }, { SI, 2 }, { DDO, 4 }, { SI, 2 }, { DDO, 2 }, { SI, 2 }, { RA, 2 }, { SI, 2 }, { DDO, 2 },
	{ RRE, 2 }, { MMI, 2 }, { FFA, 2 }, { SSOL, 2 }, { EOS, 4 } };



unsigned int seconds;		/*타이머 시간을 저장하는 전역변수*/
volatile int timer_state, buzzer_state; // 타이머와 부저의 상태를 저장하는 전역 변ㅅ

/*
  악보 재생시 필요한 전역변수
  state: 파형의 골과 마루 엽를 저장한다.
  tone : 음계 값을 저장한다. 실제 TIMER2에 설정될 카운터 수는 f_table[tone]이다.
  mute : 쉼표인지 여부를 나타낸다. 
*/

int state, tone, mute;		

// 5개의 태스크 함수 선언
void LedTask(void *data);
void TempTask(void *pdata);
void DispFndTask(void *pdata);
void WatchTask(void *pdata);
void BuzzerTask(void *pdata);

// 온도센서와의 I2C 통신을 위한 함수
void init_twi_port();
void write_twi_1byte_nopreset(char reg, char data);
int read_twi_2byte_nopreset(char reg);


/*  ----------------------------------------------------
	타이머를 잠시 멈추는 함수
	timer_state를 PAUSE로 놓고
	TIMER1A 마스크비트를 언셋하여, TIMER1 인터럽트가 발생하지 않게 한다.
	---------------------------------------------------- */
void pauseTimer()
{
	timer_state = PAUSE;
	TIMSK &= ~(1 << OCIE1A);
}

/* ----------------------------------------------------
	타이머를 리셋하는 함수
	timer_state를 OFF로 놓고
	TIMER1A 마스크비트를 언셋하여, TIMER1 인터럽트가 발생하지 않게 한다.
	설정된 타이머 값도 0으로 설정한다.
	---------------------------------------------------- */

void resetTimer()
{
	timer_state = OFF;
	seconds = 0;
	TIMSK &= ~(1 << OCIE1A);
}

/* ----------------------------------------------------
	설정된 타이머를 시작한다.
	timer_state를 ON으로 놓고
	TIMER1A 마스크비트를 세팅하여, TIMER1 인터럽트가 발생하게 한다.
	---------------------------------------------------- */
void startTimer()
{
	timer_state = ON;
	TIMSK |= 1 << OCIE1A;
}
/* ----------------------------------------------------
	알람곡 재생을 위해 부저를 켠다.
	buzzer_state를 ON으로 놓고,
	BuzzerTask를 만들어 실행시킨다.
	음악재생에 사용되는 TIMER2 마스크비트를 세팅하여, TIMER2 오버플로우 인터럽트가 발생하게 한다.
	---------------------------------------------------- */

void buzzerOn()
{
	buzzer_state = ON;
	OSTaskCreate(BuzzerTask, (void *)0, (void *)&BuzzerTaskStk[BUZZERTASK_STK_SIZE - 1], 1);

	TIMSK |= 0x40; // Overflow
}

/* ----------------------------------------------------
재생 중인 부저를 끈다.
buzzer_state를 OFF로 놓고,
BuzzerTask를 OSTaskDel로 종료시키고 삭제한다.
TIMER2 마스크비트를 언세팅하여, TIMER2 오버플로우 인터럽트가 발생되지 않게 한다.
---------------------------------------------------- */

void buzzerOff()
{
	buzzer_state = OFF;
	TIMSK &= ~(0x40); // Overflow
	OSTaskDel(BUZZ_PRIO);
}

/* ----------------------------------------------------
*	버튼 비프음 출력하는 함수
*	
---------------------------------------------------- */
void beep()
{
	int duration = 25;
	while (duration--)
	{
		PORTB = 0x10;
		_delay_ms(1);
		PORTB = 0x00;
		_delay_ms(1);
	}
}
/* ----------------------------------------------------
	TIMER1에 대한 인터럽트 서비스 루틴
	TIMER1의 값비교 인터럽트를 사용함.
	0.5초에 1회 발생
	---------------------------------------------------- */
ISR(TIMER1_COMPA_vect)
{
	INT8U	 err;
	OSFlagPost(e_grp, 0x01, OS_FLAG_SET, &err);	
}

/* ----------------------------------------------------
	1번 버튼 입력에 대한 인터럽트 처리 함수
	파워가 켜져있을 때는 타이머를 중지한다.
	타이머 값이 0이 아닐 때는 타이머를 시작한다.
--------------------------------------------------- */
ISR(INT4_vect)
{

	beep();

	if (timer_state == ON)
	{
		pauseTimer();
	}
	else if (seconds > 0)
	{
		startTimer();
	}

	_delay_ms(10); // debouncing
	
}
/*****************************************************
	2번 단추 입력에 대한 인터럽트 서비스 루틴
	버튼음을 출력한 후,
	1) 부저가 켜져 있는 상태이면 부저를 끈다.
	2) 그렇지 않은 경우는 타이머를 세팅하는 기능을 한다. 즉
	   타이머 세팅값을 10초씩 증가시킨다.
*****************************************************/
ISR(INT5_vect)
{
	beep();

	if (buzzer_state == ON)
	{
		buzzerOff();
	}
	else
	{
		seconds += 10;
		FndValue = seconds / 60 * 100 + seconds % 60;
	}
	_delay_ms(10); // debouncing

}
/****************************************************
	TIMER2에 대한 인터럽트 서비스 루틴
	TIMER2의 오버플로우 인터럽트를 사용한다.
	1. 전역변수 mute 값이 1로 설정된 경우 무음(쉼표)이므로 바로 리턴한다.
	2. state 값을 비교하여 PORTB의 5번 비트를 셋하거나 언셋한다
	3. state 값을 반대로 바꾸어 다음에 들어올 때 PORTB의 5번 비트 출력이 바뀔 수 있도록 한다.
	****************************************************/

ISR(TIMER2_OVF_vect)
{
	//INT8U	err;

	if (mute)
		return;

	if (state == ON)
	{
		PORTB = 0x00;
		state = OFF;
	}
	else
	{
		PORTB = 0x10;
		state = ON;
	}
	//OSSemPend(ToneSem, 0, &err);
	TCNT2 = f_table[tone];
	//OSSemPost(ToneSem);

}
/* ***************************************************
	프로그램 메인 함수

	1. OS가 사용할 클럭을 설정한다. 인터럽트가 걸리지 않도록 크리티컬섹션에 들어간다
	2. 프로그램에 사용될 세마포어, 이벤트그룹, 메일박스를 생성한다.
	3. 버튼 입력에 대한 인터럽트를 설정한다.
	4. 각종 디바이스 상태를 초기화 한다.
	5. BuzzerTask를 제외한 4개의 태스크를 생성한다.
	6. 멀티태스킹을 시작한다.
	
	*************************************************** */

int main (void)
{
	INT8U	 err;
	
	OSInit();

	OS_ENTER_CRITICAL();
	TCCR0=0x07;
	TIMSK =_BV(TOIE0);
	TCNT0=256-(CPU_CLOCK_HZ/OS_TICKS_PER_SEC/ 1024);
	OS_EXIT_CRITICAL();

	// 2개의 버튼에 대한 인터럽트 가능 설정
	EICRB = 0x0F;		// INT4,5 = 상승엣지 사용
	EIMSK |= 0x30;		// INT4,5 interrupt enable
	sei();
	
	// 동기화 개체 생성

	FndValSem = OSSemCreate(1);			// 전역변수 FndValue와 FndDigit변수를 보호하기 위한 세마포어
	ToneSem = OSSemCreate(1);			// Tone변수 중복 접근을 막기위한 세마포어
	e_grp = OSFlagCreate(0x00, &err);	// TIMER1 인터럽트 서비스 루틴이 인터럽트가 발생했음을 알리는 프래그
	TempMbox = OSMboxCreate((void *)0); // 온도측정하는TempTask가 온도가 한도치를 초과했는지를 메시지로 LedTask에게 보냄

	// 디바이스 상태값 초기화	
	timer_state = OFF;			// 타이머 상태 = 꺼짐
	buzzer_state = OFF;			// 부저 상태 = 꺼짐
	mute = 0;					// 쉼표, 무음모드 = 아님

	// 1개의 시간타이머 태스크와 3개의 디바이스 제어 태스크 생성
	OSTaskCreate(WatchTask, (void *)0, (void *)&WatchTaskStk[TASK_STK_SIZE - 1], WATCH_PRIO);
	OSTaskCreate(LedTask, (void *)0, (void *)&LedTaskStk[TASK_STK_SIZE - 1], LED_PRIO);
	OSTaskCreate(DispFndTask, (void *)0, (void *)&DispFndTaskStk[TASK_STK_SIZE - 1], DISP_PRIO);
	OSTaskCreate(TempTask, (void *)0, (void *)&TempTaskStk[TASK_STK_SIZE - 1], TEMP_PRIO);
	
	OSStart();		//멀티태스킹 시작

	return 0;
}

/***********************************************************************************************
	WatchTask : Timer 처리 태스크

	사용자가 입력한 시간에서 시간을 줄여나가는 카운터 기능을 하는 태스크
	
	1. TIMER1을 값비교 인터럽트로 설정한다. 분과 초사이의 점을 깜박이게
		하기 위하여 0.5초 단위로 인터럽트를 발생시킨다.
	2. resetTimer를 호출하여 타이머를 초기화한다.
	3. 무한 루프를 돌면서 다음을 반복한다.
		1) 타이머 ISR로 부터 이벤트그룹의 플래그가 세팅되길 기다린다. 0.5초가 흐른것임
		2) 짝수번째이면, FndDigit = 0x00(점 안찍음), 홀수번째이면, 
			FndDigit = 0x04(우측셋째자리에 점찍음)을 반복 출력하여, 1초 단위로 점이 깜박이게 만든다.
		3)  짝수 번째마다 seconds 를 1씩 줄이고 0이되면 buzzerOn함수를 호출하여 알람음악을 울리고,
			타이머를 0으로 리셋한다.인터럽트도 disable한다.
		4) 현재의 seconds값을 FND에 표출할 수 있도록 FndValue에 변환하여 출력한다.변환 공식은
		   seconds/60 * 100 + seconds%60 이다.

***********************************************************************************************/

void WatchTask(void *pdata)
{
	INT8U	err;
	unsigned int halfseconds = 0;

	pdata = pdata;
	
	OS_ENTER_CRITICAL();
	TCCR1B = (1 << CS12 | 1<<CS10 | 1 << WGM12);
	//OCR1A = 15625 - 1;			//1초
	OCR1A = 7813 - 1;		//0.5초
	TIMSK |= 1 << OCIE1A;
	OS_ENTER_CRITICAL();

	resetTimer();			//오븐의 타이머를 초기화한다.

	for (;;){
		//Flag가 셋팅될 때까지 기다린다.이 값은 TIMER1 ISR에서 0.5초마다 세팅한다.
		OSFlagPend(e_grp, 0x01, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, 0, &err);  
		
		halfseconds++;
		halfseconds %= 2;

		if (!halfseconds)
		{
			OSSemPend(FndValSem, 0, &err);	// DispFndTask랑 공유하므로 보호
			FndDigit = 0x00;				// 소숫점을 표시하지 않음
			OSSemPost(FndValSem);

			if ((--seconds) == 0)		//설정된 시간에 다다름.
			{
				buzzerOn();				// 알람 태스크를 생성하고 음악을 재생한다.
				resetTimer();			// 오븐의 타이머를 초기화한다
			}
			
		}
		else
		{
			OSSemPend(FndValSem, 0, &err);
			FndDigit = 0x04;				// 소숫점 표시. 1초 단위 깜박임.
			OSSemPost(FndValSem);
		}

		OSSemPend(FndValSem, 0, &err);
		FndValue = seconds/60 * 100 + seconds%60;	// 초를 분.초로 변환하여 값을 설정
		OSSemPost(FndValSem);	
		
		OSTimeDlyHMSM(0, 0, 0, 10);
	}
}

/*******************************************************************************
	BuzzerTask : 
	악보를 읽어서 TIMER2가 주어진 계명의 주파수 만큼 파형을 만들 수 있게한다.
	악보는 계명과 박자의 2차배열로 선언하였다. 전역변수 tone에 해당 계명을 저장하고,
	OSTimeDlyHMSM 함수를 통해 박자수 * 120 ms 만큼 지연시킨다.
	
*********************************************************************************/

void BuzzerTask(void *pdata)
{

	/*
	int song[29][2] = {{SOL,4}, {MI,2}, {MI,2}, {SOL,2}, {MI,2}, {DO,2},{PS,2}, 
		{RE,4}, {MI,2}, {RE,2}, {DO,2}, {MI,2}, {SOL,2},{PS,2}, 
		{DDO,3}, {SOL,1}, {DDO,2}, {SOL,2}, {DDO,2},{SOL,2}, {MI,2},{PS, 2},
		{SOL,4}, {RE,3}, {FA,1}, {MI,2}, {RE,2}, {DO,4}, {EOS, 4}};*/
	int i,j;
	INT8U err;

	i=0;

	OS_ENTER_CRITICAL();
	DDRB = 0x10;		// 부저 출력방향 설정
	TCCR2 = 0x03;		// 8분주 	
	TCNT2 = f_table[song[i][0]]; // 타이머 초기값으로 첫번째 계명에 해당하는 카운터값 입력
	TIMSK |= 0x40;		// Overflow 인터럽트로 설정
	OS_EXIT_CRITICAL();

	for (j=0;j<2 ;j++ )		// 한곡을 2번 재생
	{
	
		i =0 ;
		OSSemPend(ToneSem, 0, &err);
		tone = song[i][0];				//첫번째 계명을 읽음
		OSSemPost(ToneSem);

		while (tone != EOS)				//계명이 악보의 끝이 아닐때까지 반복
		{		
			
			mute = (tone == PS) ?  1 : 0;  //계명이 쉼표이면 전역변수 mute를 1로 설정 TIMER2 ISR에서 포트 출력을 막는다.
 			OSTimeDlyHMSM(0,0,0,120*song[i][1]); //박자수 * 120 ms만큼 지연
			mute = 1;
			OSTimeDlyHMSM(0,0,0,10);		//음이 이어지지 않도록 잠시 끊어줌
			mute = 0;
			i++;							// 다음계명을 읽어와서
			OSSemPend(ToneSem, 0, &err);
			tone = song[i][0];				// ISR이 읽어 갈수 있도록 tone 변수에 세팅
			OSSemPost(ToneSem);
		}	
	} 			

	buzzerOff();		//2번 재생이 완료되면 태스크를 종료시키고, 인터럽트를 disable한다.
	
}

/*****************************************************************************************************
	LedTask : 가열판을 작동시키는 태스크
	1. INT4에 의해 가동 혹은 중지임이 설정된 timer_state를 읽어  ON일 경우 가열판(LED)를 작동시킨다.
	2. 과열방지를 위해 과열정보를 TempMox에서 OSMboxAccept(TempMbox)함수로 읽어온다. TempTask는 0.5초 마다
	온도센서에서 온도값을 읽어 임계치 보다 큰지의 여부를 OSMboxPost를 통해 전송해온다.
	3. 과열이 아니거나 타이머가 ON중이면 LED 움직이는 패턴을 출력한다.
*******************************************************************************************************/

void LedTask (void *pdata)
{
	unsigned char direction = 0xff;
	unsigned char bit = 0x07;
	int bimetal, *msg;
	
	pdata = pdata;

	bimetal = ON;						// 과열 아님
	DDRA = 0xff;						// LED PORTA 출력으로 설정

	for(;;)
	{
		msg = (int *)OSMboxAccept(TempMbox);	//TempMbox에서 wait 없이 읽어옴. LED의 원활한 출력을 위해
		if (msg != 0)
		{
			bimetal = *msg;				//과열 정보 수신
		}
		
		// LED display

		if ((timer_state == ON) && (bimetal == ON))	//과열이 아니면서 타이머가 ON중이면 LED 움직이는 패턴을 출력한다.
		{
			PORTA = bit;
			if(bit == 0x07 || bit == 0xE0)			// 3비트씩 연속으로 좌우로 움직이는 패턴 표시
				direction = ~direction;
			bit = direction? (bit >> 1) : (bit << 1);
		}
		else
			PORTA = 0;								// 꺼졌거나 과열이면 가열판을 끈다.
	
		OSTimeDlyHMSM(0,0,0,100);
	}
}

/*
	TempTask : 온도센서에서 온도를 읽어 과열여부를 판단하는 TASK

	1. I2C 통신을 통해 온도센서에서 온도값을 읽어와 실수로 변환한다.
	2. 시연의 용이성을 현재의 온도 + 0.5도를 threshold(임계값)으로 사용한다.
	3. 무한루프를 돌면서 ATS75로 부터 온도 값을 읽어 임계값을 지나쳤는지 확인한다.
	4. Tempbox 메시지박스에 포스팅하여, LEDTask가 과열 여부를 알 수 있도록 한다.
*/

void TempTask(void *pdata) {
	int value;
	float temperature, threshold ;
	unsigned short value_int, value_deci;
	int bimetal;

	init_twi_port(); // TWI 및 포트 초기화
	
	write_twi_1byte_nopreset(ATS75_CONFIG_REG, 0x00); // 9비트, Normal
	OSTimeDlyHMSM(0, 0, 0,100); // 다음 사이클을 위하여 잠시 기다림
	
	value = read_twi_2byte_nopreset(ATS75_TEMP_REG);
	value_int = (char)((value & 0x7f00) >> 8);
	value_deci = (char)(value & 0x00ff);
	temperature = value_int + ((value_deci & 0x80) == 0x80) * 0.5;  //측정값을 실수로 변환
	threshold = temperature + 0.5;						//현재값 + 0.5를 임계치로, 0.5도만 상승해도 꺼지도록
	OSTimeDlyHMSM(0, 0, 0, 500);						// 0.5초지연
	
	while (1) {			// 무한 루프를 돌면서 온도값을 읽어 과열여부를 판단후 Mbox에 포스팅한다.
		value = read_twi_2byte_nopreset(ATS75_TEMP_REG);
		value_int = (char)((value & 0x7f00) >> 8);
		value_deci = (char)(value & 0x00ff);

		temperature = value_int + ((value_deci & 0x80) == 0x80) * 0.5;
		
		bimetal = (temperature > threshold) ? OFF : ON;
		//FndValue = value_int;

		OSMboxPost(TempMbox,(void*)&bimetal);
		OSTimeDlyHMSM(0, 0, 0,500);
	}
}

/*
	DispFndTask :  지속적으로 FND에 숫자를 출력하는 태스크
	1. 태스크 시작과 함께 PORTC와 PORTG의 출력방향을 아웃풋으로 설정한다.
	2. 세마포어 획득 후 WatchTask와 ISR5가 설정한 FndValue와 FndDIgit을 읽어와 FND에 출력한다
	3. FndDigit bit의 값에 따라 소숫점을 찍는다.
*/

void DispFndTask(void *pdata)
{
	int i;
	unsigned short num[4];
	int value;
	unsigned char point;
	INT8U	err;

	DDRC = 0xff;		// PORTC 출력
	DDRG = 0x0f;		// PROTG 출력

	while (1)
	{
		OSSemPend(FndValSem, 0, &err);	//세마포어 획득
		value = FndValue;				//전역변수를 읽음
		point = FndDigit;
		OSSemPost(FndValSem);

		num[3] = (value / 1000) % 10;	// 각 자리수 구함
		num[2] = (value / 100) % 10;
		num[1] = (value / 10) % 10;
		num[0] = value % 10;

		// 이자리에 출력
		for (i = 0; i < 4; i++) {
			PORTC = digit[num[i]];		//해당 숫자에 해당하는 비트열 출력
			PORTG = fnd_sel[i];
			if (point & BIT(i))			//FndDigit의 해당 자리 비트가 세팅되어 있으면 점 출력
				PORTC |= 0x80;
			OSTimeDlyHMSM(0, 0, 0,2);
		}
		//OSTimeDlyHMSM(0, 0, 0, 50);
	}
}

/*
	I2C 통신 출력함수들
*/
void init_twi_port() {
	//DDRC = 0xff;
	//DDRG = 0xff; // FND 출력 세팅
	PORTD = 3; // For Internal pull-up for SCL & SCK
	SFIOR &= ~(1 << PUD); // PUD = 0 : Pull Up Disable
	TWBR = (F_CPU / F_SCK - 16) / 2; // 공식 참조, bit trans rate 설정
	TWSR = TWSR & 0xfc; // Prescaler 값 = 00 (1배)
}
void write_twi_1byte_nopreset(char reg, char data) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);// START 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x08); // START 상태 검사, 이후 모두 상태 검사
	TWDR = ATS75_ADDR | 0; // SLA+W 준비, W=0
	TWCR = (1 << TWINT) | (1 << TWEN); // SLA+W 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18);
	TWDR = reg; // aTS75 Reg 값 준비
	TWCR = (1 << TWINT) | (1 << TWEN); // aTS75 Reg 값 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28);
	TWDR = data; // DATA 준비
	TWCR = (1 << TWINT) | (1 << TWEN); // DATA 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28);
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP 전송
}

int read_twi_2byte_nopreset(char reg) {
	char high_byte, low_byte;
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);// START 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x08); // START 상태 검사, 이후 ACK 및 상태 검사
	TWDR = ATS75_ADDR | 0; // SLA+W 준비, W=0
	TWCR = (1 << TWINT) | (1 << TWEN); // SLA+W 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18);
	TWDR = reg; // aTS75 Reg 값 준비
	TWCR = (1 << TWINT) | (1 << TWEN); // aTS75 Reg 값 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28);
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);// RESTART 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x10); // RESTART 상태 검사, 이후 ACK, NO_ACK 상태 검사
	TWDR = ATS75_ADDR | 1; // SLA+R 준비, R=1
	TWCR = (1 << TWINT) | (1 << TWEN); // SLA+R 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x40);
	TWCR = (1 << TWINT) | (1 << TWEN | 1 << TWEA);// 1st DATA 준비
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x50);
	high_byte = TWDR; // 1st DATA 수신
	TWCR = (1 << TWINT) | (1 << TWEN);// 2nd DATA 준비
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x58);
	low_byte = TWDR; // 2nd DATA 수신
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP 전송
	return((high_byte << 8) | low_byte); // 수신 DATA 리턴
}

