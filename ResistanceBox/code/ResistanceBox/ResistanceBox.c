/*
 * ResistanceBox.c
 *
 * Created: 2014-02-28 11:27:19
 *  Author: terg
 */ 

#define F_CPU	1000000UL

#include <avr/io.h>
#include <util/delay.h>

#define STROBE_ON	PORTC |=  0x08
#define STROBE_OFF	PORTC &=  0xF7

void init(){
	
	//------------------------------4094-----------------------
	//Port C as output for 4094
	DDRC  |= 0x0F;
	//STORBE=0		DATA=0		CLK=1		OE=1
	PORTC |= 0x03;
	
	//-----------------------------4543------------------------
	DDRB  |=  0x01;	//PB0 output
	PORTB &= ~0x01;	//PB0 = 0
	DDRD  |=  0xE0;	//PB7-5 output
	PORTD &= ~0xE0;	//PB7-5 = 0
	//-----------------------------L293------------------------
	DDRD |=  0x1C;	//PD2-4 output
	PORTD &= ~0x1C;	//PD2-4 set to 0
	//---------------------7SEG display select-----------------
	DDRB  |= 0x7E;
	PORTB &= ~0x7E;
	//----------------------------74HC165----------------------
	DDRC |= 0x20;	//PC5 = output			CLK
	DDRC &= ~0x10;	//PC4 = input			DATA
	DDRB |= 0x80;	//PB7 = output			LOAD
	//---------------------CLEAR THE 4094 OUTPUTS--------------
	clearResistance();
	STROBE_ON;
}

/*------------------------------------------------
				   L293 - H BRIDGE
------------------------------------------------*/
void H_Bridge(int data){
	//data = -1 turn off relay
	//data = 0 turn off h bridge
	//data = 1 turn on relay
	switch (data){
		case -1:
			PORTD |=  0x0C;		//EN = 1		IN1=1
			PORTD &= ~0x10;		//IN2 = 0
			break;
		case 0:
			PORTD &= ~0x1C;		//EN = 0		IN1 = 0			IN2 = 0
			break;
		case 1:
			PORTD |= 0x14;		//EN = 1		IN2 = 1
			PORTD &= ~0x08;		//IN1 = 0
			break;
	}
}


/*------------------------------------------------
						4094
------------------------------------------------*/
void sendBitTo4094(uint8_t data){
	PORTC |= 0x02;	//CLK up
	
	if(data)	//set DATA pin
		PORTC |= 0x04;
	else
		PORTC &= 0xFB;
	
	PORTC &= 0xFD;	//CLK down
	PORTC |= 0x02;	//CLK up
}

void sendTo4094(uint8_t data){
	for (int i=7;i>=0;i--){
		PORTC |= 0x02;	//CLK up
		
		if(data & (1<<i))	//set DATA pin
			PORTC |= 0x04;
		else
			PORTC &= 0xFB;
		
		PORTC &= 0xFD;	//CLK down
	}
	PORTC |= 0x02;	//CLK up
}

/*The fastest way. It doesnt need double numbers opposite to math.h*/
uint32_t pow10(uint8_t numb){
	switch (numb){
		case 0:
			return 1; break;
		case 1:
			return 10; break;
		case 2:
			return 100; break;
		case 3:
			return 1000; break;
		case 4:
			return 10000; break;
		case 5:
			return 100000; break;
		case 6:
			return 1000000; break;
	}
	return 1;
}

uint8_t convertDigitToRelay(uint8_t digit){
	uint8_t relayState=0;	//1-> means relay on
	
	if((digit % 2)==0)	//even number
		relayState += 1;	//relay5 ON
	
	if(digit < 8)
		relayState += 32;		//relay0 ON
	else
		return relayState;
	
	if(digit > 5){
		relayState+=16;		//relay1 ON
		return relayState;
	}
	if(digit > 3){
		relayState+=8;		//relay2 ON
		return relayState;
	}
	if(digit > 1){
		relayState += 4;		//relay3 ON
		return relayState;
	}
	
	relayState += 2;	//relay4 ON
	return relayState;
}

void clearResistance(){
	STROBE_OFF;
	for(int i=5;i>=0;i--)
		sendTo4094(0x00);
	STROBE_ON;
	
	H_Bridge(-1);
	_delay_ms(50);
	
	//2x1 going through all relays
	sendBitTo4094(1);
	for (int i=0;i<6*8;i++){		//6*8 dla jednej jedynki
		_delay_ms(2);
		sendBitTo4094(0);
	}
	
	_delay_ms(2);
	
	/*STROBE_OFF;
	for(int i=8;i>=0;i--)
		sendTo4094(0x01);
	STROBE_ON;
	for(int i=6;i>=0;i--){
		sendBitTo4094(0);
		_delay_ms(4);
	}		*/
	
	H_Bridge(0);
}

void setResistance(uint32_t resistance){
	clearResistance();
	
	H_Bridge(1);
	/*STROBE_OFF;
	uint8_t digit=0;
	//go through every digit in the resistance value
	for (int i=5;i>=0;i--){
		digit = (resistance % pow10(i+1))/pow10(i);
		sendTo4094(convertDigitToRelay(digit));
	}
	STROBE_ON;*/
	
	_delay_ms(5);
	
	resistance=resistance/100;
	
	uint8_t digit=0;
	//go through every digit in the resistance value
	for (int i=5;i>=0;i--){
		STROBE_OFF;
		
		for (int ii=8;ii>=0;ii--){
			sendTo4094(0x00);
		}
		
		digit = (resistance % pow10(i+1))/pow10(i);
		sendTo4094(convertDigitToRelay(digit));
		
		for (int ii=i-1;ii>=0;ii--){
			sendTo4094(0x00);
		}
		
		STROBE_ON;
		_delay_ms(2);
	}
	
	
	//after setting the 4094 it is set up for some time to change the resistance
	//_delay_ms(5);
	
	H_Bridge(0);
	
	STROBE_OFF;
	//clearResistance();
	STROBE_ON;
}

/*------------------------------------------------
				   7SEG CONTROLLER
------------------------------------------------*/

void writeNumberOn7Seg(int display, long int number){
	
	//clear7Seg();
	PORTB &= ~0x7E;
	
	for(int i=0;i<8*6;i++)
		sendBitTo4094(0);
	STROBE_ON;
	
	uint8_t numberToDisp=0;
	
	//w³aczenie odpowiedniego wyswietlacza
	//STROBE_OFF;
	//sendBitTo4094(1);
	//sendBitTo4094(1);
	switch(display){
		case 0:
			numberToDisp = number%10L;
			PORTB |= 0x10;	//ok
			break;
		case 1:
			numberToDisp = (number%100L)/10L;
			PORTB |= 0x20;
			break;
		case 2:
			numberToDisp = (number%1000L)/100L;
			PORTB |= 0x40;
			break;
		case 3:
			numberToDisp = (number%10000L)/1000L;
			PORTB |= 0x08;	//ok
			break;
		case 4:
			numberToDisp = (number%100000L)/10000L;
			PORTB |= 0x04;	//ok
			break;
		case 5:
			numberToDisp = (number%1000000L)/100000L;
			PORTB |= 0x02;	//ok
			break;
	}
	//STROBE_OFF;
	
	//A
	if(numberToDisp&0x01)
		PORTD |= 0x20;	//PD5
	else
		PORTD &= ~0x20;
	
	//B
	if(numberToDisp&0x02)
		PORTD |= 0x80;	//PD7
	else
		PORTD &= ~0x80;
	
	//C
	if(numberToDisp&0x04)
		PORTB |= 0x01;	//PB0
	else
		PORTB &= ~0x01;
	
	//D
	if(numberToDisp&0x08)
		PORTD |= 0x40;	//PD6
	else
		PORTD &= ~0x40;
}

/*------------------------------------------------
					   74HC165
------------------------------------------------*/

uint8_t readDataFrom74HC165(){
	uint8_t data=0;
	PORTB |= 0x80;	//LOAD=1
	
	for(uint8_t i=0;i<8;i++){
		PORTC &= ~0x20;		//CLK low
		if (!(PINC&0x10))	//DATA = 0 button is pressed
		data+=(1<<i);
		
		PORTC |= 0x20;		//CLK high
	}
	PORTB &= ~0x80;	//LOAD=0
	return (data&0x8F);
	
	/*
	0x08 ===  OK button
	0x04 ===   ^ UP
	0x02 ===   --> Right
	0x01 ===   <-- Left
	0x80 ===   V DOWN
	*/
}

/*------------------------------------------------
						MAIN
------------------------------------------------*/
int main(void)
{
	
	init();
	
	long int resistance=0;
	//setResistance(resistance);
	_delay_ms(100);
	
	int iDisp=0;
	int selectDigit=-1;	//-1 means no change
	int selectDigitCounter=0;
	int keyPress=0;
	
	int testCounter=0;
	
	clearResistance();
	
	_delay_ms(100);
	
    while(1)
    {
		H_Bridge(0);	//to make sure that any relay isnt on
        if(iDisp==selectDigit && selectDigitCounter<50)
			PORTB &= ~0x3E; //clear7Seg();
        else
			writeNumberOn7Seg(iDisp,resistance);
		
		//--------------buttons---------------
		uint8_t button = readDataFrom74HC165();
		if(keyPress==0){
			if(selectDigit!=-1){
			
				//Down button
				if(button&0x80){
					switch (selectDigit){
						case 0:
						resistance--; break;
						case 1:
						resistance-=10; break;
						case 2:
						resistance-=100; break;
						case 3:
						resistance-=1000L; break;
						case 4:
						resistance-=10000L; break;
						case 5:
						resistance-=100000L; break;
					}
					if(resistance<0)
						resistance=0;
					keyPress=50;
				}
			
				//UP button
				else if(button&0x04){
					switch (selectDigit){
						case 0:
						resistance++; break;
						case 1:
						resistance+=10; break;
						case 2:
						resistance+=100; break;
						case 3:
						resistance+=1000L; break;
						case 4:
						resistance+=10000L; break;
						case 5:
						resistance+=100000L; break;
					}
					if(resistance>999999L)
						resistance=999999L;
					keyPress=50;
				}
		
				//LEFT button
				else if(button&0x01){
					if(selectDigit!=5)
						selectDigit++;
					else
						selectDigit=0;
					keyPress=50;
				}
	
				//RIGHT button
				else if(button&0x02){
					if(selectDigit!=0)
						selectDigit--;
					else
						selectDigit=5;
					keyPress=50;
				}

				//OK Button
				else if(button&0x08){
					selectDigit=-1;
					keyPress=50;
					setResistance(resistance);
				}
			}
			else {
				if(button&0x08){
					selectDigit=0;
					keyPress=50;
				}
			}
		}
		else{
			keyPress--;
		}	
				
		//---------------delay----------------
		_delay_ms(1);
		
		//--------------counters--------------
		iDisp++;
		if(iDisp==6)
			iDisp=0;
			
		selectDigitCounter++;
		if(selectDigitCounter==100)
			selectDigitCounter=0;	
			
		/*testCounter++;
		if(testCounter==500){
			if(resistance<1000){
				resistance+=100;
			}
			else
				resistance+=1000L;
			if(resistance>9999)
				resistance=0;
			testCounter=0;
			setResistance(resistance);
		}*/
    }
}