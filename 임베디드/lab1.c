#include "includes.h"
#define F_CPU	16000000UL	// CPU frequency = 16 Mhz
#include <avr/io.h>	
#include <util/delay.h>
#include <avr/interrupt.h>

#define TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE      
#define ATS75_ADDR 0x98 // 0b10011000, 7��Ʈ�� 1��Ʈ left shift
#define ATS75_CONFIG_REG 1
#define ATS75_TEMP_REG 0
#define F_SCK 40000UL // SCK Ŭ�� �� = 40 Khz

#define BIT(x) (1 << x)
#define BUZZERTASK_STK_SIZE 512

// �� �½�ũ�� �켱���� ����
#define BUZZ_PRIO	1
#define WATCH_PRIO	2
#define LED_PRIO	3
#define TEMP_PRIO	4
#define DISP_PRIO	5

// ���� ����
#define ON 0
#define OFF 1
#define PAUSE 2

//����� 
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
�½�ũ ���� : 5�� �½�ũ�� ���� ���ñ���ü�̴�. Buzzer �½�ũ�� �Ǻ� ������ ũ�� ��Ƴ��Ҵ�.
*/
OS_STK	LedTaskStk[TASK_STK_SIZE];
OS_STK	TempTaskStk[TASK_STK_SIZE];
OS_STK  DispFndTaskStk[TASK_STK_SIZE];
OS_STK  WatchTaskStk[TASK_STK_SIZE];
OS_STK  BuzzerTaskStk[512];

/*
���α׷��� ���Ǵ� ��� ����ȭ ��ü�� ������.
2���� ��������, 1���� ���Ϲڽ�, 1���� �̺�Ʈ �׷��� ����Ͽ���.
*/
OS_EVENT     *FndValSem;
OS_EVENT     *ToneSem;
OS_EVENT	 *TempMbox;
OS_FLAG_GRP  *e_grp;


// FND ����Ʈ ������ PORTG�� ���� Selection bit 
unsigned char digit[12] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x27, 0x7f, 0x6f, 0x40, 0x00 };
unsigned char fnd_sel[4] = { 0x01, 0x02, 0x04, 0x08 };
/*
   FND�� ǥ�õ� ���� ���� ��������
   DispFnd �½�ũ�� ���� ������ ���鼭 �Ʒ� ���� FND�� ����Ѵ�.
   FndDigit�� �Ҽ����� ���� �ڸ��� ���� ��Ʈ���� �����Ѵ�. 
   0x01 = �� ���� FND�� �Ҽ��� ǥ��, 0x0F = ��� �ڸ��� �Ҽ��� ǥ��
*/
int FndValue;
unsigned char FndDigit;

//�� ������ Ÿ�̸� �����÷ο� ���ͷ�Ʈ�� ���� �ʱ� Ÿ�̸� ���̴�.
char f_table[12] = {17, 43, 66, 77, 97, 114, 117, 137, 150, 161, 167, 176 };
// �˶����ǿ� ���� �Ǻ��̴�. {���, ����}�� 2���� �迭�� �����Ͽ���. 
int song[57][2] = {
	{ SSOL, 4 }, { MMI, 2 }, { FFA, 2 }, { SSOL, 4 }, { MMI, 2 }, { FFA, 2 }, { SSOL, 2 },
	{ SI, 2 }, { RA, 2 }, { SI, 2 }, { DDO, 2 }, { RRE, 2 }, { MMI, 2 }, { FFA, 2 },
	{ MMI, 4 }, { DDO, 2 }, { RRE, 2 }, { MMI, 4 }, { MI, 4 }, { FA, 2 }, { SOL, 2 }, { RA, 2 },
	{ SOL, 2 }, { FA, 2 }, { SOL, 2 }, { DDO, 2 }, { SI, 2 }, { DDO, 2 },

	{ RA, 4 }, { DDO, 2 }, { SI, 2 }, { RA, 4 }, { SOL, 2 }, { FA, 2 }, { SOL, 2 }, { FA, 2 },
	{ MI, 2 }, { FA, 2 }, { SOL, 2 }, { RA, 2 }, { SI, 2 }, { DDO, 2 }, { RA, 4 },
	{ DDO, 2 }, { SI, 2 }, { DDO, 4 }, { SI, 2 }, { DDO, 2 }, { SI, 2 }, { RA, 2 }, { SI, 2 }, { DDO, 2 },
	{ RRE, 2 }, { MMI, 2 }, { FFA, 2 }, { SSOL, 2 }, { EOS, 4 } };



unsigned int seconds;		/*Ÿ�̸� �ð��� �����ϴ� ��������*/
volatile int timer_state, buzzer_state; // Ÿ�̸ӿ� ������ ���¸� �����ϴ� ���� ����

/*
  �Ǻ� ����� �ʿ��� ��������
  state: ������ ��� ���� ���� �����Ѵ�.
  tone : ���� ���� �����Ѵ�. ���� TIMER2�� ������ ī���� ���� f_table[tone]�̴�.
  mute : ��ǥ���� ���θ� ��Ÿ����. 
*/

int state, tone, mute;		

// 5���� �½�ũ �Լ� ����
void LedTask(void *data);
void TempTask(void *pdata);
void DispFndTask(void *pdata);
void WatchTask(void *pdata);
void BuzzerTask(void *pdata);

// �µ��������� I2C ����� ���� �Լ�
void init_twi_port();
void write_twi_1byte_nopreset(char reg, char data);
int read_twi_2byte_nopreset(char reg);


/*  ----------------------------------------------------
	Ÿ�̸Ӹ� ��� ���ߴ� �Լ�
	timer_state�� PAUSE�� ����
	TIMER1A ����ũ��Ʈ�� ����Ͽ�, TIMER1 ���ͷ�Ʈ�� �߻����� �ʰ� �Ѵ�.
	---------------------------------------------------- */
void pauseTimer()
{
	timer_state = PAUSE;
	TIMSK &= ~(1 << OCIE1A);
}

/* ----------------------------------------------------
	Ÿ�̸Ӹ� �����ϴ� �Լ�
	timer_state�� OFF�� ����
	TIMER1A ����ũ��Ʈ�� ����Ͽ�, TIMER1 ���ͷ�Ʈ�� �߻����� �ʰ� �Ѵ�.
	������ Ÿ�̸� ���� 0���� �����Ѵ�.
	---------------------------------------------------- */

void resetTimer()
{
	timer_state = OFF;
	seconds = 0;
	TIMSK &= ~(1 << OCIE1A);
}

/* ----------------------------------------------------
	������ Ÿ�̸Ӹ� �����Ѵ�.
	timer_state�� ON���� ����
	TIMER1A ����ũ��Ʈ�� �����Ͽ�, TIMER1 ���ͷ�Ʈ�� �߻��ϰ� �Ѵ�.
	---------------------------------------------------- */
void startTimer()
{
	timer_state = ON;
	TIMSK |= 1 << OCIE1A;
}
/* ----------------------------------------------------
	�˶��� ����� ���� ������ �Ҵ�.
	buzzer_state�� ON���� ����,
	BuzzerTask�� ����� �����Ų��.
	��������� ���Ǵ� TIMER2 ����ũ��Ʈ�� �����Ͽ�, TIMER2 �����÷ο� ���ͷ�Ʈ�� �߻��ϰ� �Ѵ�.
	---------------------------------------------------- */

void buzzerOn()
{
	buzzer_state = ON;
	OSTaskCreate(BuzzerTask, (void *)0, (void *)&BuzzerTaskStk[BUZZERTASK_STK_SIZE - 1], 1);

	TIMSK |= 0x40; // Overflow
}

/* ----------------------------------------------------
��� ���� ������ ����.
buzzer_state�� OFF�� ����,
BuzzerTask�� OSTaskDel�� �����Ű�� �����Ѵ�.
TIMER2 ����ũ��Ʈ�� �����Ͽ�, TIMER2 �����÷ο� ���ͷ�Ʈ�� �߻����� �ʰ� �Ѵ�.
---------------------------------------------------- */

void buzzerOff()
{
	buzzer_state = OFF;
	TIMSK &= ~(0x40); // Overflow
	OSTaskDel(BUZZ_PRIO);
}

/* ----------------------------------------------------
*	��ư ������ ����ϴ� �Լ�
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
	TIMER1�� ���� ���ͷ�Ʈ ���� ��ƾ
	TIMER1�� ���� ���ͷ�Ʈ�� �����.
	0.5�ʿ� 1ȸ �߻�
	---------------------------------------------------- */
ISR(TIMER1_COMPA_vect)
{
	INT8U	 err;
	OSFlagPost(e_grp, 0x01, OS_FLAG_SET, &err);	
}

/* ----------------------------------------------------
	1�� ��ư �Է¿� ���� ���ͷ�Ʈ ó�� �Լ�
	�Ŀ��� �������� ���� Ÿ�̸Ӹ� �����Ѵ�.
	Ÿ�̸� ���� 0�� �ƴ� ���� Ÿ�̸Ӹ� �����Ѵ�.
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
	2�� ���� �Է¿� ���� ���ͷ�Ʈ ���� ��ƾ
	��ư���� ����� ��,
	1) ������ ���� �ִ� �����̸� ������ ����.
	2) �׷��� ���� ���� Ÿ�̸Ӹ� �����ϴ� ����� �Ѵ�. ��
	   Ÿ�̸� ���ð��� 10�ʾ� ������Ų��.
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
	TIMER2�� ���� ���ͷ�Ʈ ���� ��ƾ
	TIMER2�� �����÷ο� ���ͷ�Ʈ�� ����Ѵ�.
	1. �������� mute ���� 1�� ������ ��� ����(��ǥ)�̹Ƿ� �ٷ� �����Ѵ�.
	2. state ���� ���Ͽ� PORTB�� 5�� ��Ʈ�� ���ϰų� ����Ѵ�
	3. state ���� �ݴ�� �ٲپ� ������ ���� �� PORTB�� 5�� ��Ʈ ����� �ٲ� �� �ֵ��� �Ѵ�.
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
	���α׷� ���� �Լ�

	1. OS�� ����� Ŭ���� �����Ѵ�. ���ͷ�Ʈ�� �ɸ��� �ʵ��� ũ��Ƽ�ü��ǿ� ����
	2. ���α׷��� ���� ��������, �̺�Ʈ�׷�, ���Ϲڽ��� �����Ѵ�.
	3. ��ư �Է¿� ���� ���ͷ�Ʈ�� �����Ѵ�.
	4. ���� ����̽� ���¸� �ʱ�ȭ �Ѵ�.
	5. BuzzerTask�� ������ 4���� �½�ũ�� �����Ѵ�.
	6. ��Ƽ�½�ŷ�� �����Ѵ�.
	
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

	// 2���� ��ư�� ���� ���ͷ�Ʈ ���� ����
	EICRB = 0x0F;		// INT4,5 = ��¿��� ���
	EIMSK |= 0x30;		// INT4,5 interrupt enable
	sei();
	
	// ����ȭ ��ü ����

	FndValSem = OSSemCreate(1);			// �������� FndValue�� FndDigit������ ��ȣ�ϱ� ���� ��������
	ToneSem = OSSemCreate(1);			// Tone���� �ߺ� ������ �������� ��������
	e_grp = OSFlagCreate(0x00, &err);	// TIMER1 ���ͷ�Ʈ ���� ��ƾ�� ���ͷ�Ʈ�� �߻������� �˸��� ������
	TempMbox = OSMboxCreate((void *)0); // �µ������ϴ�TempTask�� �µ��� �ѵ�ġ�� �ʰ��ߴ����� �޽����� LedTask���� ����

	// ����̽� ���°� �ʱ�ȭ	
	timer_state = OFF;			// Ÿ�̸� ���� = ����
	buzzer_state = OFF;			// ���� ���� = ����
	mute = 0;					// ��ǥ, ������� = �ƴ�

	// 1���� �ð�Ÿ�̸� �½�ũ�� 3���� ����̽� ���� �½�ũ ����
	OSTaskCreate(WatchTask, (void *)0, (void *)&WatchTaskStk[TASK_STK_SIZE - 1], WATCH_PRIO);
	OSTaskCreate(LedTask, (void *)0, (void *)&LedTaskStk[TASK_STK_SIZE - 1], LED_PRIO);
	OSTaskCreate(DispFndTask, (void *)0, (void *)&DispFndTaskStk[TASK_STK_SIZE - 1], DISP_PRIO);
	OSTaskCreate(TempTask, (void *)0, (void *)&TempTaskStk[TASK_STK_SIZE - 1], TEMP_PRIO);
	
	OSStart();		//��Ƽ�½�ŷ ����

	return 0;
}

/***********************************************************************************************
	WatchTask : Timer ó�� �½�ũ

	����ڰ� �Է��� �ð����� �ð��� �ٿ������� ī���� ����� �ϴ� �½�ũ
	
	1. TIMER1�� ���� ���ͷ�Ʈ�� �����Ѵ�. �а� �ʻ����� ���� �����̰�
		�ϱ� ���Ͽ� 0.5�� ������ ���ͷ�Ʈ�� �߻���Ų��.
	2. resetTimer�� ȣ���Ͽ� Ÿ�̸Ӹ� �ʱ�ȭ�Ѵ�.
	3. ���� ������ ���鼭 ������ �ݺ��Ѵ�.
		1) Ÿ�̸� ISR�� ���� �̺�Ʈ�׷��� �÷��װ� ���õǱ� ��ٸ���. 0.5�ʰ� �帥����
		2) ¦����°�̸�, FndDigit = 0x00(�� ������), Ȧ����°�̸�, 
			FndDigit = 0x04(������°�ڸ��� ������)�� �ݺ� ����Ͽ�, 1�� ������ ���� �����̰� �����.
		3)  ¦�� ��°���� seconds �� 1�� ���̰� 0�̵Ǹ� buzzerOn�Լ��� ȣ���Ͽ� �˶������� �︮��,
			Ÿ�̸Ӹ� 0���� �����Ѵ�.���ͷ�Ʈ�� disable�Ѵ�.
		4) ������ seconds���� FND�� ǥ���� �� �ֵ��� FndValue�� ��ȯ�Ͽ� ����Ѵ�.��ȯ ������
		   seconds/60 * 100 + seconds%60 �̴�.

***********************************************************************************************/

void WatchTask(void *pdata)
{
	INT8U	err;
	unsigned int halfseconds = 0;

	pdata = pdata;
	
	OS_ENTER_CRITICAL();
	TCCR1B = (1 << CS12 | 1<<CS10 | 1 << WGM12);
	//OCR1A = 15625 - 1;			//1��
	OCR1A = 7813 - 1;		//0.5��
	TIMSK |= 1 << OCIE1A;
	OS_ENTER_CRITICAL();

	resetTimer();			//������ Ÿ�̸Ӹ� �ʱ�ȭ�Ѵ�.

	for (;;){
		//Flag�� ���õ� ������ ��ٸ���.�� ���� TIMER1 ISR���� 0.5�ʸ��� �����Ѵ�.
		OSFlagPend(e_grp, 0x01, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, 0, &err);  
		
		halfseconds++;
		halfseconds %= 2;

		if (!halfseconds)
		{
			OSSemPend(FndValSem, 0, &err);	// DispFndTask�� �����ϹǷ� ��ȣ
			FndDigit = 0x00;				// �Ҽ����� ǥ������ ����
			OSSemPost(FndValSem);

			if ((--seconds) == 0)		//������ �ð��� �ٴٸ�.
			{
				buzzerOn();				// �˶� �½�ũ�� �����ϰ� ������ ����Ѵ�.
				resetTimer();			// ������ Ÿ�̸Ӹ� �ʱ�ȭ�Ѵ�
			}
			
		}
		else
		{
			OSSemPend(FndValSem, 0, &err);
			FndDigit = 0x04;				// �Ҽ��� ǥ��. 1�� ���� ������.
			OSSemPost(FndValSem);
		}

		OSSemPend(FndValSem, 0, &err);
		FndValue = seconds/60 * 100 + seconds%60;	// �ʸ� ��.�ʷ� ��ȯ�Ͽ� ���� ����
		OSSemPost(FndValSem);	
		
		OSTimeDlyHMSM(0, 0, 0, 10);
	}
}

/*******************************************************************************
	BuzzerTask : 
	�Ǻ��� �о TIMER2�� �־��� ����� ���ļ� ��ŭ ������ ���� �� �ְ��Ѵ�.
	�Ǻ��� ���� ������ 2���迭�� �����Ͽ���. �������� tone�� �ش� ����� �����ϰ�,
	OSTimeDlyHMSM �Լ��� ���� ���ڼ� * 120 ms ��ŭ ������Ų��.
	
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
	DDRB = 0x10;		// ���� ��¹��� ����
	TCCR2 = 0x03;		// 8���� 	
	TCNT2 = f_table[song[i][0]]; // Ÿ�̸� �ʱⰪ���� ù��° ��� �ش��ϴ� ī���Ͱ� �Է�
	TIMSK |= 0x40;		// Overflow ���ͷ�Ʈ�� ����
	OS_EXIT_CRITICAL();

	for (j=0;j<2 ;j++ )		// �Ѱ��� 2�� ���
	{
	
		i =0 ;
		OSSemPend(ToneSem, 0, &err);
		tone = song[i][0];				//ù��° ����� ����
		OSSemPost(ToneSem);

		while (tone != EOS)				//����� �Ǻ��� ���� �ƴҶ����� �ݺ�
		{		
			
			mute = (tone == PS) ?  1 : 0;  //����� ��ǥ�̸� �������� mute�� 1�� ���� TIMER2 ISR���� ��Ʈ ����� ���´�.
 			OSTimeDlyHMSM(0,0,0,120*song[i][1]); //���ڼ� * 120 ms��ŭ ����
			mute = 1;
			OSTimeDlyHMSM(0,0,0,10);		//���� �̾����� �ʵ��� ��� ������
			mute = 0;
			i++;							// ��������� �о�ͼ�
			OSSemPend(ToneSem, 0, &err);
			tone = song[i][0];				// ISR�� �о� ���� �ֵ��� tone ������ ����
			OSSemPost(ToneSem);
		}	
	} 			

	buzzerOff();		//2�� ����� �Ϸ�Ǹ� �½�ũ�� �����Ű��, ���ͷ�Ʈ�� disable�Ѵ�.
	
}

/*****************************************************************************************************
	LedTask : �������� �۵���Ű�� �½�ũ
	1. INT4�� ���� ���� Ȥ�� �������� ������ timer_state�� �о�  ON�� ��� ������(LED)�� �۵���Ų��.
	2. ���������� ���� ���������� TempMox���� OSMboxAccept(TempMbox)�Լ��� �о�´�. TempTask�� 0.5�� ����
	�µ��������� �µ����� �о� �Ӱ�ġ ���� ū���� ���θ� OSMboxPost�� ���� �����ؿ´�.
	3. ������ �ƴϰų� Ÿ�̸Ӱ� ON���̸� LED �����̴� ������ ����Ѵ�.
*******************************************************************************************************/

void LedTask (void *pdata)
{
	unsigned char direction = 0xff;
	unsigned char bit = 0x07;
	int bimetal, *msg;
	
	pdata = pdata;

	bimetal = ON;						// ���� �ƴ�
	DDRA = 0xff;						// LED PORTA ������� ����

	for(;;)
	{
		msg = (int *)OSMboxAccept(TempMbox);	//TempMbox���� wait ���� �о��. LED�� ��Ȱ�� ����� ����
		if (msg != 0)
		{
			bimetal = *msg;				//���� ���� ����
		}
		
		// LED display

		if ((timer_state == ON) && (bimetal == ON))	//������ �ƴϸ鼭 Ÿ�̸Ӱ� ON���̸� LED �����̴� ������ ����Ѵ�.
		{
			PORTA = bit;
			if(bit == 0x07 || bit == 0xE0)			// 3��Ʈ�� �������� �¿�� �����̴� ���� ǥ��
				direction = ~direction;
			bit = direction? (bit >> 1) : (bit << 1);
		}
		else
			PORTA = 0;								// �����ų� �����̸� �������� ����.
	
		OSTimeDlyHMSM(0,0,0,100);
	}
}

/*
	TempTask : �µ��������� �µ��� �о� �������θ� �Ǵ��ϴ� TASK

	1. I2C ����� ���� �µ��������� �µ����� �о�� �Ǽ��� ��ȯ�Ѵ�.
	2. �ÿ��� ���̼��� ������ �µ� + 0.5���� threshold(�Ӱ谪)���� ����Ѵ�.
	3. ���ѷ����� ���鼭 ATS75�� ���� �µ� ���� �о� �Ӱ谪�� �����ƴ��� Ȯ���Ѵ�.
	4. Tempbox �޽����ڽ��� �������Ͽ�, LEDTask�� ���� ���θ� �� �� �ֵ��� �Ѵ�.
*/

void TempTask(void *pdata) {
	int value;
	float temperature, threshold ;
	unsigned short value_int, value_deci;
	int bimetal;

	init_twi_port(); // TWI �� ��Ʈ �ʱ�ȭ
	
	write_twi_1byte_nopreset(ATS75_CONFIG_REG, 0x00); // 9��Ʈ, Normal
	OSTimeDlyHMSM(0, 0, 0,100); // ���� ����Ŭ�� ���Ͽ� ��� ��ٸ�
	
	value = read_twi_2byte_nopreset(ATS75_TEMP_REG);
	value_int = (char)((value & 0x7f00) >> 8);
	value_deci = (char)(value & 0x00ff);
	temperature = value_int + ((value_deci & 0x80) == 0x80) * 0.5;  //�������� �Ǽ��� ��ȯ
	threshold = temperature + 0.5;						//���簪 + 0.5�� �Ӱ�ġ��, 0.5���� ����ص� ��������
	OSTimeDlyHMSM(0, 0, 0, 500);						// 0.5������
	
	while (1) {			// ���� ������ ���鼭 �µ����� �о� �������θ� �Ǵ��� Mbox�� �������Ѵ�.
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
	DispFndTask :  ���������� FND�� ���ڸ� ����ϴ� �½�ũ
	1. �½�ũ ���۰� �Բ� PORTC�� PORTG�� ��¹����� �ƿ�ǲ���� �����Ѵ�.
	2. �������� ȹ�� �� WatchTask�� ISR5�� ������ FndValue�� FndDIgit�� �о�� FND�� ����Ѵ�
	3. FndDigit bit�� ���� ���� �Ҽ����� ��´�.
*/

void DispFndTask(void *pdata)
{
	int i;
	unsigned short num[4];
	int value;
	unsigned char point;
	INT8U	err;

	DDRC = 0xff;		// PORTC ���
	DDRG = 0x0f;		// PROTG ���

	while (1)
	{
		OSSemPend(FndValSem, 0, &err);	//�������� ȹ��
		value = FndValue;				//���������� ����
		point = FndDigit;
		OSSemPost(FndValSem);

		num[3] = (value / 1000) % 10;	// �� �ڸ��� ����
		num[2] = (value / 100) % 10;
		num[1] = (value / 10) % 10;
		num[0] = value % 10;

		// ���ڸ��� ���
		for (i = 0; i < 4; i++) {
			PORTC = digit[num[i]];		//�ش� ���ڿ� �ش��ϴ� ��Ʈ�� ���
			PORTG = fnd_sel[i];
			if (point & BIT(i))			//FndDigit�� �ش� �ڸ� ��Ʈ�� ���õǾ� ������ �� ���
				PORTC |= 0x80;
			OSTimeDlyHMSM(0, 0, 0,2);
		}
		//OSTimeDlyHMSM(0, 0, 0, 50);
	}
}

/*
	I2C ��� ����Լ���
*/
void init_twi_port() {
	//DDRC = 0xff;
	//DDRG = 0xff; // FND ��� ����
	PORTD = 3; // For Internal pull-up for SCL & SCK
	SFIOR &= ~(1 << PUD); // PUD = 0 : Pull Up Disable
	TWBR = (F_CPU / F_SCK - 16) / 2; // ���� ����, bit trans rate ����
	TWSR = TWSR & 0xfc; // Prescaler �� = 00 (1��)
}
void write_twi_1byte_nopreset(char reg, char data) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);// START ����
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x08); // START ���� �˻�, ���� ��� ���� �˻�
	TWDR = ATS75_ADDR | 0; // SLA+W �غ�, W=0
	TWCR = (1 << TWINT) | (1 << TWEN); // SLA+W ����
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18);
	TWDR = reg; // aTS75 Reg �� �غ�
	TWCR = (1 << TWINT) | (1 << TWEN); // aTS75 Reg �� ����
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28);
	TWDR = data; // DATA �غ�
	TWCR = (1 << TWINT) | (1 << TWEN); // DATA ����
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28);
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP ����
}

int read_twi_2byte_nopreset(char reg) {
	char high_byte, low_byte;
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);// START ����
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x08); // START ���� �˻�, ���� ACK �� ���� �˻�
	TWDR = ATS75_ADDR | 0; // SLA+W �غ�, W=0
	TWCR = (1 << TWINT) | (1 << TWEN); // SLA+W ����
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18);
	TWDR = reg; // aTS75 Reg �� �غ�
	TWCR = (1 << TWINT) | (1 << TWEN); // aTS75 Reg �� ����
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28);
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);// RESTART ����
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x10); // RESTART ���� �˻�, ���� ACK, NO_ACK ���� �˻�
	TWDR = ATS75_ADDR | 1; // SLA+R �غ�, R=1
	TWCR = (1 << TWINT) | (1 << TWEN); // SLA+R ����
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x40);
	TWCR = (1 << TWINT) | (1 << TWEN | 1 << TWEA);// 1st DATA �غ�
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x50);
	high_byte = TWDR; // 1st DATA ����
	TWCR = (1 << TWINT) | (1 << TWEN);// 2nd DATA �غ�
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x58);
	low_byte = TWDR; // 2nd DATA ����
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // STOP ����
	return((high_byte << 8) | low_byte); // ���� DATA ����
}

