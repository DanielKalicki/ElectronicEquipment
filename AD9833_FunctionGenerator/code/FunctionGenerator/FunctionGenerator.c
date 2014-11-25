/*
 * FunctionGenerator.c
 *
 * Created: 2014-05-01 15:55:09
 *  Author: terg
 */ 

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define UART_BAUD_RATE      38400

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "ad9833.h"
#include "spi.h"

//--------------------------LCD-------------------------
void writeDataTo74HC595(uint8_t E,uint8_t RS, uint8_t D4, uint8_t D5, uint8_t D6, uint8_t D7){
	for (int i=7;i>=0;i--){
		PORTC &= ~0x20;	//Shift clock down
		
		uint8_t bitData=0;
		switch (i){
			case 7:
				bitData=D7; break;
			case 6:
				bitData=D6; break;
			case 5:
				bitData=D5;	break;
			case 4:
				bitData=D4; break;
			case 2:
				bitData=RS; break;
			case 1:
				bitData=E; break;
		}
		if(bitData)	//set DATA pin
			PORTC |= 0x08;
		else
			PORTC &= ~0x08;
		
		PORTC |= 0x20;	//shift clock up => Shift the register
	}
	PORTC &= ~0x20;	//shift clock down
	
	PORTC &= ~0x10;	//strobe clock down;
	PORTC |= 0x10;	//strobe clock up    =>  output data from register on the pins outputs
}

void writeToLCD_Data(uint8_t RS, uint8_t D4, uint8_t D5, uint8_t D6, uint8_t D7){
	writeDataTo74HC595(1,RS,D4,D5,D6,D7);
	writeDataTo74HC595(0,RS,D4,D5,D6,D7);
}

void writeToLCD_Letter(uint8_t ascii){
	writeToLCD_Data(1,ascii&0x10,ascii&0x20,ascii&0x40,ascii&0x80);
	writeToLCD_Data(1,ascii&0x01,ascii&0x02,ascii&0x04,ascii&0x08);
}

void clearLCD(){
	writeToLCD_Data(0,0,0,0,0);
	writeToLCD_Data(0,1,0,0,0);
}

void secondLineLCD(){
	writeToLCD_Data(0,0,0,1,1);
	writeToLCD_Data(0,0,0,0,0);
}

void initLCD(){
	_delay_ms(100);
	
	writeToLCD_Data(0,0,1,0,0);		//set 4-bit mode
	_delay_ms(5); // czekaj 5ms
	
	writeToLCD_Data(0,0,1,0,0);		//set 4-bit mode
	_delay_ms(5); // czekaj 5ms
	
	writeToLCD_Data(0,0,0,0,1);		//set number of lines and font character
	_delay_ms(5); // czekaj 5ms
	
	writeToLCD_Data(0,0,0,0,0);	//Display on/off control
	writeToLCD_Data(0,0,0,1,1);
	
	clearLCD();
}

void initADC(){
	//ADC
	// Wybranie zewnêtrzego ¿ród³a napiêcia odniesienia
	ADMUX |=  _BV(REFS0);
	ADMUX &= ~_BV(REFS1);
	//  Wybranie ADC6
	ADMUX &= ~_BV(MUX3);
	ADMUX |= _BV(MUX2);
	ADMUX |= _BV(MUX1);
	ADMUX &= ~_BV(MUX0);
	//	Wybranie sposobu zapisu wyniku z wyrównaniem do lewej (osiem starszych bitów wyniku w rejestrze ADCH)
	ADMUX |= _BV(ADLAR);

	ADCSRA |= _BV(ADEN);	//ADC enabled
	// Wybranie czêstotliwoœci dla taktowania przetwornika
	ADCSRA |= _BV(ADPS0);		//wybranie prescalera 128
	ADCSRA |= _BV(ADPS1);
	ADCSRA |= _BV(ADPS2);
	
	ADCSRA |= _BV(ADSC);
	
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU)); 				// w³¹czenie modu³u UART, ustawienie prêdkoœci transmisji
	sei();
	uart_putc('A');
}

void init(){
	//UART
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));
	sei();
	
	//SPI
	spi_init(SPIMODE2);
	
	//buttons
	/*
			PB7	=	SQUARE
			PB6	=	TRIANGLE
			PD5	=	SIN
	*/
	DDRB &= ~0xC0;	//PB6 & PB7 as input
	PORTB |= 0xC0;	//PB6 & PB7 pull-up resistors
	DDRD &= ~0x20;	//PD5 as input
	PORTD |= 0x20;	//PD5 pull-up resistors
	
	//diodes
	/*
			PD6	=	SIN
			PD7	=	SQUARE
			PB0	=	TRIANGLE
	*/
	DDRD |= 0xC0;	//PD7 & PD6 as outputs
	PORTD &= ~0xC0;	//PD7 & PD6 low
	DDRB |= 0x01;	//PB0 as output
	PORTB &= ~0x01;	//PB0 low
	
	//Shift buttons
	DDRB &= ~0x02;	//PB1 as input for ">"
	PORTB |= 0x02;	//PB1 with pull-up resistor
	
	//Select Register button
	DDRC &= ~0x02;	//PC1 as input
	PORTC |= 0x02;	//PC1 with a pull-up resistor
	
	//Select Register diode
	DDRC |= 0x01;	//PC0 as output
	PORTC &= ~0x01;	//PC0 set low
	
	//encoder
	DDRD &= ~0x18;	//PD3 & PD4 as input
	PORTD |= 0x18;	//PD3 & PD4 with pull-up resistor
	
	//LCD
	DDRC |= 0x38;	//set PC5,4,3 as outputs
	PORTC &= ~0x38;	//set PC5,4,3 to low
	
	ad9833_init();
	initADC();
	initLCD();
}

void clearDiodes(){
	PORTD &= ~0xC0;
	PORTB &= ~0x01;
}

uint8_t phaseFreq=1;
uint8_t chanNumber=1;
uint8_t selectRegister=0;
uint8_t selectedMode=AD_OFF;

void checkButtons(){
	static unsigned int timer=0;
	if(timer!= 0) timer++;
	if(timer>=5000)	timer=0;
	
	if(timer==0){
		//SQUARE pressed
		if(!(PINB & 0x80)){
			clearDiodes();
			if(selectedMode!=AD_SQUARE)
				PORTD |= 0x80;	//PD7 high
			else {
				if(phaseFreq==0)	{ phaseFreq=1; PORTD |= 0x80; }
				else phaseFreq=0;
			}
			timer=1;
			selectedMode=AD_SQUARE;
			ad9833_set_mode(AD_SQUARE);
		}
		//TRIANGLE pressed
		else if(!(PINB & 0x40)){
			clearDiodes();
			if(selectedMode!=AD_TRIANGLE)
				PORTB |= 0x01;	//PB0 high
			else {
				if(phaseFreq==0)	{ phaseFreq=1; PORTB |= 0x01; }
				else phaseFreq=0;
			}
			timer=1;
			selectedMode=AD_TRIANGLE;
			ad9833_set_mode(AD_TRIANGLE);
		}
		//SIN pressed
		else if(!(PIND & 0x20)){
			clearDiodes();
			if(selectedMode!=AD_SINE)
				PORTD |= 0x40;	//PD6 high
			else {
				if(phaseFreq==0)	{ phaseFreq=1; PORTD |= 0x40;}
				else phaseFreq=0;
			}				
			timer=1;
			selectedMode=AD_SINE;
			ad9833_set_mode(AD_SINE);
		}
		
		else if(!(PINB & 0x02)){	// > button
			if(chanNumber==0)
				chanNumber=7;
			else
				chanNumber--;
			timer=1;
		}
		
		else if(!(PINC & 0x02))	{//select register button
			if(selectRegister==0){
				selectRegister=1;
				PORTC |= 0x01;	//diode on
			}
			else {
				selectRegister=0;
				PORTC &= ~0x01;
			}
			ad9833_set_freq_out(selectRegister);
			timer=1;
		}			
	}	
}

//returns -1/1/0
int encoder(){
	static char prevA=0;	//PD3
	
	static char prev=0;
	
	int ret=0;
	if(prevA != (PIND & 0x08)) {
		if(prev==0x10 && (PIND & 0x18)==0x18) ret = -1;
		if(prev==0x00  && (PIND & 0x18)==0x18) ret = 1;
		
		prev = (PIND & 0x18);
	}	
	
	prevA = (PIND & 0x08);
	
	return ret;
}

void sendToLCD(char buff[40]){
	for(int i=0;i<40;i++){
		if(buff[i]==0) break;
		if(buff[i]=='\n') secondLineLCD();
		else writeToLCD_Letter(buff[i]);
	}		
}

unsigned long int myPow10(int selectDigit){
	switch (selectDigit){
		case 0:
			return 1; break;
		case 1:
			return 10; break;
		case 2:
			return 100; break;
		case 3:
			return 1000; break;
		case 4:
			return 10000L; break;
		case 5:
			return 100000L; break;
		case 6:
			return 1000000L; break;
		case 7:
			return 10000000L; break;
	}
	return 0;
}

int main(void)
{
	init();
	uart_putc('s');
	
	unsigned long int freq[2]={0,50};	//freq in Hz * 10
	unsigned int phase[2]={0,0};	//phase in degress * 10
	
	char buffer[40];
	
	_delay_ms(100);
	
	unsigned int time=0;
	
	uint8_t dataFromADC=0;
	
    while(1)
    {
		if(bit_is_clear(ADCSRA,ADSC)){
			dataFromADC = ADCH;
			ADCH=0;
			ADCL=0;
			ADCSRA |= (1<<ADSC)|(1<<ADEN);
		}
		time++;
		if(time==10000){
			time=0;
			if(phaseFreq==0){
				switch (selectedMode){
					case AD_TRIANGLE:
						PORTD ^= 0x01;
						break;
					case AD_SQUARE:
						PORTB ^= 0x80;
						break;
					case AD_SINE:
						PORTD ^= 0x40;
						break;
				}
			}
		}			
			
		if(time%2500==0){
			clearLCD();
			_delay_ms(2);
			sprintf(buffer,"%ld.%d", (long int)freq[selectRegister]/10, (int)(freq[selectRegister]%10));
			//sendToLCD(buffer);
			writeToLCD_Letter('f');
			writeToLCD_Letter(selectRegister+48);
			writeToLCD_Letter(':');
			writeToLCD_Letter(' ');
			uint8_t wNumb=0;
			uint8_t num=0;
			for(int i=7;i>=0;i--){
				if(i==0)
					writeToLCD_Letter('.');
				if(i==1) 
					wNumb=1;
				num = (freq[selectRegister]/myPow10(i))%10;
				if(num!=0){
					wNumb=1;
					if(chanNumber==i && time==5000)
						writeToLCD_Letter('_');
					else writeToLCD_Letter(num+48);
				}					
				else{
					if(chanNumber==i && time==5000)
						writeToLCD_Letter('_');
					else {
						if(wNumb) writeToLCD_Letter('0');
						else writeToLCD_Letter(' ');
					}						
				}
			}
			secondLineLCD();
			sprintf(buffer,"ADC: %d mV\n",dataFromADC*19);
			sendToLCD(buffer);
			
			/*int digit=0;
			for (digit=0;digit<10;digit++)
				if(buffer[digit]==0 || buffer[digit]=='\n') break;
			for (int i=0;i<11-digit;i++){
				if(time==5000 && 11-chanNumber-2==i)
					writeToLCD_Letter('_');
				else writeToLCD_Letter(' ');
			}				
			sendToLCD(buffer);*/
		}
		checkButtons();
		
		int enc = encoder();
		if(!(enc==-1 && freq[selectRegister]==0)){
			long int powTen = myPow10(chanNumber);
			if(phaseFreq==1){
				if(enc==-1 && powTen > freq[selectRegister])  freq[selectRegister]=0;
				else freq[selectRegister]+=enc*powTen;
				if(freq[selectRegister] >= 100000000L) freq[selectRegister] = 100000000L;
			}
			else {
				if(enc==-1 && powTen > phase[selectRegister])  phase[selectRegister]=0;
				phase[selectRegister]+=enc*powTen;	//test only
				if(phase[selectRegister]>=3650) phase[selectRegister]-=3650;
			}	
		}
		if(enc!=0){
			if(phaseFreq==1)
				ad9833_set_frequency(selectRegister,freq[selectRegister]/10.0);
		}		
		
    }
}