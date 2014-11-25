/*
 * PowerSupply.c
 *
 * Created: 2013-03-03 12:42:55
 *  Author: Daniel
 */ 

// czêstotliwoœæ CPU w MHz
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// prêdkoœæ transmisji 38400
#define UART_BAUD_RATE      9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
#include "i2cmaster.h"
#include <stdio.h>

volatile uint16_t voltageLevel=0;	//liczba od 0 do 9999
volatile uint16_t actualVoltage=0;
volatile uint16_t AddingNumber=1; //liczba ktora dodajemy co przesuniecie enkodera

#define DAC_ADDRESS  0xC0

unsigned char read_i2c(unsigned char address){
	unsigned char data=0;
	if (i2c_start(DAC_ADDRESS+I2C_WRITE))  {i2c_stop();	return 0;} 				// set device address and write mode
	i2c_write(address);						// write address = i_address
	i2c_rep_start(DAC_ADDRESS+I2C_READ);				// set device address and read mode
	data = i2c_readNak();								// read one byte form address 0
	i2c_stop();
	return data;
}

unsigned char write_i2c(unsigned int dataV, unsigned int dataI){
	
	if(i2c_start(DAC_ADDRESS+I2C_WRITE)) {i2c_stop();	return 0;}     // set device address and write mode
	i2c_write(0x0A);
	i2c_write(0xFF);                        //channel A
	
	i2c_write(0x30);	//power down mode
	i2c_write(0x00);                        //channel B
	
	i2c_write(0x0F&((uint8_t)(dataV/256)));
	i2c_write(dataV%256);                   //channel C
	
	i2c_write(0x30);	//power down mode
	i2c_write(0x00);                        //channel D
	i2c_stop();
	
	return 1;				//success
}

void initPowerSupply(){
	//ustawienie portów jako wyjœæ do sterowania wyœwietlaczem 7seg.
	DDRC |= _BV(3);
	DDRC |= _BV(4);
	DDRC |= _BV(5);
	DDRC |= _BV(6);
	//ustawienie portów jako wyjœæ do wyboru wyœwietlacza 7seg. stan wysoki oznacza ze wyswietlacz jest aktywny
	DDRB |= _BV(0);
	DDRB |= _BV(1);
	DDRB |= _BV(2);
	DDRB |= _BV(3);
	
	//wejscia na przyciski steruj¹ce
	DDRB &= ~_BV(4);
	DDRB &= ~_BV(5);
	//pulups
	PORTB |= _BV(4);
	PORTB |= _BV(5);
	
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));
	
	//Timer ktory pisze na 7seg kolejne liczby
	TCCR0 |= (1<<CS00) | (1<<CS01); //ustawienie preskalera na 1024
	TIMSK |= 1<<TOIE0; //w³¹czenie przerwania od przepe³nienia licznika
	TCNT0 = 0; //ustawienie wartoœci pocz¹tkowej
	
	//przerwania od enkodera napiecia
	//MCUCR |=  (1<<ISC00)|(1<<ISC01);			//wlaczenie przerwania od INT0 zboczem narastaj¹cym
	MCUCR |= (1<<ISC01); MCUCR &= ~(1<<ISC00);	//przerwanie zboczem opadajacym
	GICR |= (1<<INT0);							//przerwanie od INT0
	
	//ustawnienie jako wejsc portów z dodatkowa informacja z enkoder
	DDRD &= ~_BV(4);
	DDRD &= ~_BV(3);
	
	/* http://www.voytek.evbox.pl/programy/adc/Przetwornik_AC.html */
	// Wybranie wewnetrznego ¿ród³a napiêcia odniesienia 5V
	ADMUX |= _BV(REFS0);
	ADMUX &= ~_BV(REFS1);
	//  Wybranie PA5
	ADMUX |= _BV(MUX0);
	ADMUX &= ~_BV(MUX1);
	ADMUX |= _BV(MUX2);
	//	Wybranie sposobu zapisu wyniku z wyrównaniem do lewej (osiem starszych bitów wyniku w rejestrze ADCH)
	//ADMUX |= _BV(ADLAR);
	// Zezwolenie na konwersjê
	ADCSRA |= _BV(ADEN);
	// Wybranie czêstotliwoœci dla taktowania przetwornika
	ADCSRA |= _BV(ADPS0);		//wybranie prescalera 128
	ADCSRA |= _BV(ADPS1);
	ADCSRA |= _BV(ADPS2);
	
	sei();
	
	i2c_init();
	if(i2c_start(DAC_ADDRESS+I2C_WRITE)) {i2c_stop(); uart_putc('n');}			//warunek prawdziwy nie ma eepromu
	

}
/*
Display means which 7segment (digit) you want to turn on. Numers are from 0 to 3.
Number is a value which will be display on 7seg.
*/
void writeNumberOn7Seg(uint8_t display, uint8_t number){
	for(uint8_t i=0;i<4;i++) {
		if((display)==i) PORTB |= _BV(display);
		else PORTB &= ~_BV(i);
	}	
	
	if(number&(1<<0)) PORTC |= _BV(6);
	else PORTC &= ~_BV(6);
	
	if(number&(1<<1)) PORTC |= _BV(4);
	else PORTC &= ~_BV(4);
	
	if(number&(1<<2)) PORTC |= _BV(3);
	else PORTC &= ~_BV(3);
	
	if(number&(1<<3)) PORTC |= _BV(5);
	else PORTC &= ~_BV(5);
}

int main(void)
{
	initPowerSupply();
	
	uart_putc('b');

	voltageLevel=0;		//moze odczytana z ADC
	
	ADCSRA |= _BV(ADSC);     		    // Rozpoczêcie przetwarzania
	
	int i_voltageADC=0;
	long int mean_voltageADC=0;
	
	int AddingNumberPress=0;
	
    while(1)
    {
		if(bit_is_clear(ADCSRA,ADSC)){
			i_voltageADC++;
			mean_voltageADC+=ADC;
			if(i_voltageADC==6){
				//actualVoltage=(uint16_t)((5.0/256.0)*(double)ADCH*5.0*10.0);
				long int dataFromADC = (float)(mean_voltageADC/6+3)*500*5;
				uint16_t newVoltage = (uint16_t)(dataFromADC/1024);
				if(newVoltage>actualVoltage+3 || newVoltage<actualVoltage-3)	//histereza
					actualVoltage=newVoltage;
				mean_voltageADC=0;
				i_voltageADC=0;
			}				
			ADC=0;
			ADCSRA |= _BV(ADSC);     		    // Rozpoczêcie przetwarzania
		}			
        write_i2c(voltageLevel,1);
		unsigned int c = uart_getc();
		if(c=='a') {//prosba o aktualny stan napiecia wyjsciowego
			char buff[20];
			sprintf(buff,"%d\n",actualVoltage);
			uart_puts(buff);
		}
		if(c=='o') {//prosba o ustawiony stan napiecia wyjsciowego
			char buff[20];
			sprintf(buff,"%d\n",voltageLevel*5+45);
			uart_puts(buff);
		}
		if(c=='i'){
			unsigned int cc[2]={0,0};
			int i=0;
			for (int y=0;y<30000;++y) asm("nop");
			for (i=0;i<2;++i){
				cc[i] = uart_getc();
				if(cc[i] & UART_NO_DATA)
					i=20;		//wyjœcie z pêtli for.		i=20 oznacza ze tutaj wystapil jakiœ b³¹d i dane nie powinny byæ przetwarzane
				else for (int y=0;y<30000;++y) asm("nop");
			}
			if(i!=20){
				char buff[20];
				sprintf(buff,"%d\n",cc[0]*256+cc[1]);
				uart_puts(buff);
				int NewData = ((int)(cc[0]*256+cc[1])-45)/5;
				if(NewData>=0){
					voltageLevel = (uint16_t)(NewData);
				}
				else {voltageLevel=0;}	
			}
			else uart_putc('e');
		}
		
		if(AddingNumberPress==0){
			if(!(PINB & (1 << 4))){
				if(AddingNumber<1000) AddingNumber*=10;
				AddingNumberPress=1000;
				char buff[20];
				sprintf(buff,"+  %d\n",AddingNumber);
				uart_puts(buff);
			}
		}	
		if(AddingNumberPress==0){
			if(!(PINB & (1 << 5))){
				if(AddingNumber>1) AddingNumber/=10;
				AddingNumberPress=1000;
				char buff[20];
				sprintf(buff,"-  %d\n",AddingNumber);
				uart_puts(buff);
			}
		}
		if(AddingNumberPress!=0) AddingNumberPress--;
    }
}

volatile uint8_t dispNumber=0;

volatile uint8_t counter=0; //tylko aby degubowac nastepnie usunac

volatile uint8_t waitEncoder;

SIGNAL(SIG_OVERFLOW0) {
	
	if(waitEncoder) {	//sofwarre dalay on encoders debuncing
		waitEncoder++;
		if(waitEncoder==10)
			waitEncoder=0;
	}
	unsigned int numberToDisplay=actualVoltage;

	uint8_t dontDispNumber=0;
	
	if(dispNumber==0){
		if ((uint8_t)((numberToDisplay%10000)/1000)!=0)
			writeNumberOn7Seg(dispNumber,(numberToDisplay%10000)/1000);
		else {dispNumber++; dontDispNumber=1;}
	}
	if(dispNumber==1){
		if(dontDispNumber){
			if((uint8_t)((numberToDisplay%1000)/100)!=0)
				writeNumberOn7Seg(dispNumber,(numberToDisplay%1000)/100);
			else dispNumber++;
		}
		else writeNumberOn7Seg(dispNumber,(numberToDisplay%1000)/100);
	}		
	if(dispNumber==2)
		writeNumberOn7Seg(dispNumber,(numberToDisplay%100)/10);
	if(dispNumber==3)
		writeNumberOn7Seg(dispNumber,(numberToDisplay%10));
	
	dispNumber++;
	if(dispNumber==4) dispNumber=0;
}	

//informacje z enkoder napiecia
ISR(INT0_vect, ISR_BLOCK) {
	if(!waitEncoder){
		if(PIND & (1 << 4)){
			voltageLevel+=AddingNumber;
		}
		else {
			if(voltageLevel>=AddingNumber)	//aby nie zejsc ponizej zera
				voltageLevel-=AddingNumber;
		}
	}	
	waitEncoder=1;
}	
	