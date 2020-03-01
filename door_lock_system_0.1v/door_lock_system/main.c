/*
 * door_lock_system.c
 *
 * Created: 2020-02-29 오후 2:48:09
 * Author : chw91
 */ 

/*
TIM1 : SERVO
TIM0 : BUZZER
UART1 : BT
UART0 : Touch Sensor (USB to Serial : UART0)

GPIOA : LED (RGB? >> PWM을 통해 신호를 주면 확실하긴 함 )
*/
#define F_CPU 16000000UL

#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD-1)

typedef unsigned char u8;

struct{
	volatile u8 rxFlag; // uart received flag
	volatile u8 rxBuf[50], txBuf[50];
	volatile u8 rxCnt, txCnt,txCntMax;
	volatile u8 rxTimeoutCnt;
}uartValue;

volatile unsigned char RxData;
//--------------------------------
//UART Rx Interrupt
//Rx부분은 아직 해결 못하겠음. >> 해결 함. (레지스터 세팅 부분에서 비트연산자 실수했음.)
ISR(USART0_RX_vect)
{
	RxData=UDR0;
}
//--------------------------------

//--------------------------------
//UART Tx Interrupt 송신 성공하면 인터럽트 발생.
/*
ISR(USART0_TX_vect)
{
	if(u0.txCnt >= u0.txCntMax)  // x>=3 (x=1,2,3) 1일땐 이미 보낸 상태.
	{
		UCSR0B &= ~0x40; //Tx 송신완료 interrupt disabled.
	}
	else UDR0 = u0.txBuf[u0.txCnt++]; // 0은 이미 보냈고 1이 보내지고 2로 증가, 2보내지고 3으로 증가되고 다음 루틴에서 인터럽트 비활성화
}*/

//똑같은 방식이지만, 문자열을 받고 나서 문자열이 끝나는 부분에 대한 비교 후 인터럽트를 꺼주는 방식.
ISR(USART0_TX_vect)
{
	if(uartValue.txBuf[uartValue.txCnt]=='\0') //문자열 마지막부분은 보내지 않는 것이 특징.
	{
		UCSR0B &= ~0x40; //Tx 송신완료 interrupt disabled.
	}
	else UDR0 = uartValue.txBuf[uartValue.txCnt++]; // 0은 이미 보냈고 1이 보내지고 2로 증가, 2보내지고 3으로 증가되고 다음 루틴에서 인터럽트 비활성화
}
//--------------------------------
void setup();

//TIM
void setOCRA(int num);
void setOCRB(int num);
void setICR(int num);
int convertServoAngle(int angle);
/*
SERVO MOTOR
Control Period : 20ms
angle : 1~2ms  >> OCR 2000~4000
*/
//UART
void UART0_Init(unsigned int ubrr);
void uartTx(char * data);
unsigned char uartRx(void);
int stack=0;




int main(void)
{
	setup();
    /* Replace with your application code */
	setOCRA(convertServoAngle(360));

	uartTx("1");
    while (1) 
    {
		_delay_ms(25);
		stack++;
		stack%=2;
		if(RxData=='1')uartTx("push"); //
		//
		//// UDR0 레지스터는 읽으면 버퍼가 지워지는게 FIFO와 흡사하기 때문에 if 문에 쓰기 부적합함을 인지하고 있자.
		
		//else putch('n');; 
			
    }
}

void setup()
{
	//SREG |= 1<<7; 
	sei();
	DDRB = 0xff; 
	
	
	//UART0
	UART0_Init(MYUBRR);
	
	//TIM0
	//16Mhz > 1024 prescailing >>15.6kHz/count >> 0.064ms >> 16.38ms(pwm period)
	TCCR0 = ((1<<CS00) | (1<< CS01) | (1<<CS02) | (1<<WGM00)|(1<<WGM01) |(1<<COM01));
	
	//TIM1
	//COM : 10 : default HIGH, compare LOW 
	//WGM : 1110 : FAST PWM, TOP:ICR1
	TCCR1A = ((1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(0<<WGM10));
	TCCR1B = ((0<<CS12)|(1<<CS11)|(0<<CS10)|(1<<WGM13)|(1<<WGM12) );
	
	TCNT0=0;
	TCNT1H = 0;
	TCNT1L = 0;
	
	setICR(39999);//0x9c3f
	
}
void setOCRA(int num)
{
	OCR1AH = (unsigned char)(num>>8);
	OCR1AL = (unsigned char)(num&0xff);
	
}

void setOCRB(int num)
{
	OCR1BH = (unsigned char)(num>>8);
	OCR1BL = (unsigned char)(num&0xff);
	
}

void setICR(int num)
{
	//39999 == 0x9c3f
	//high write first
	ICR1H = (unsigned char)(num>>8);
	ICR1L = (unsigned char)(num&0xff);
	

}

int convertServoAngle(int angle)
{
	//0~180 degree
	int converted = 2000+angle*11;
	return converted;
}


void UART0_Init(unsigned int ubrr)
{
	
	DDRE = 0b00000010;
	//set Baud Rate
	UBRR0H= (unsigned char )(ubrr>>8);
	UBRR0L= (unsigned char )(ubrr&0xff);
	
	// Tx, Rx Enabled
	UCSR0B = ((1<<TXEN0) | (1<<RXEN0));
	UCSR0B |= ((1<<RXCIE0)); //수신 완료 인터럽트 허용
	
	// Set Frame format: data 8 bit, 1 stop bit
	UCSR0C = (0<<USBS0) |(1<<UCSZ01)| (1<<UCSZ00 );
}

//uartTx("test");
void uartTx(char * data)
{
	int _len=strlen(data);
	strncpy((char*)uartValue.txBuf,data,_len);
	
	//uartValue.txBuf[0]='t';
	//uartValue.txBuf[1]='e';
	//uartValue.txBuf[2]='s';
	//uartValue.txBuf[3]='t';
	//uartValue.txBuf[4]= 0 ;
	// _len = 4 >> 문자열이므로 +1해줘야 함.
	
	while(!(UCSR0A & (1<<UDRE0)));
	//UDR0=data;
	UDR0=uartValue.txBuf[0];
	
	uartValue.txCnt=1;    //ISR상에서 사용할 cnt 시작 값
	uartValue.txCntMax=_len+1; //ISR상에서 사용할 cnt max 값 (문자열 뒤에 붙는 \0도 포함)
	//cm0.txCntMax=5;
	UCSR0B |= 0x40; //송신완료 인터럽트 활성화
	//UCSR0A |= (1<<UDRE0);
}


unsigned char uartRx(void)
{
	unsigned char data;
	while(!(UCSR0A&(1<<RXC0)));
	data=UDR0;
	//UCSR0A |= (1<<RXC0);
	return data;
}