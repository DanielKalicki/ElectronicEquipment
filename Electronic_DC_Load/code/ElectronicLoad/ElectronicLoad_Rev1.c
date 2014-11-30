/*
 * ElectronicLoad.c
 *
 * Created: 2014-03-23 11:15:36
 *  Author: Daniel
 */ 

#ifndef F_CPU
#define F_CPU	16000000UL
#endif

#define UART_BAUD_RATE      56000


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include "uart.h"
#include "i2cmaster.h"
#include <stdio.h>

//--------------------------SPI-------------------------
#define SPI_PORT	PORTB
#define SPI_DIR		DDRB
#define SPI_PIN		PINB

#define SPI_SCK		PB7
#define SPI_MISO	PB6
#define SPI_MOSI	PB5
#define SPI_CS		PB4

static inline void spi_init()
{
	SPI_DIR |= (_BV(SPI_CS) | _BV(SPI_SCK) | _BV(SPI_MOSI));
	SPI_DIR &= ~_BV(SPI_MISO);
	SPCR |= (1<<MSTR);               // Set as Master
	SPCR |= (1<<SPR0);//|(1<<SPR1);  // divided clock by 128
	SPCR |= (1<<SPE);                // Enable SPI
}

static inline uint8_t spi_data_send(uint8_t data)
{
	SPDR = data;                 // send the data
	while(!(SPSR & (1<<SPIF)));
	SPSR |= _BV(SPIF);
	return SPDR;
}

//-----------------------DAC----------------------------
void writeToDAC(uint8_t DAC, uint16_t value){

	SPI_PORT &= ~(_BV(SPI_CS));
	
	uint8_t data1 = 0x00;	//C3=0 C2=0 C1=0 C0=0 Write to Input Register
	uint8_t data2 = (value>>12);
	if (DAC == 0)
		data2 = data2&0x0F;
	else
		data2 = (data2&0x0F) + 0x30;
	uint8_t data3 = value>>4;
	uint8_t data4 = (value&0x0F)<<4;
	
	spi_data_send(data1);
	spi_data_send(data2);
	spi_data_send(data3);
	spi_data_send(data4);
	
	SPI_PORT |= (_BV(SPI_CS));
}

void powerDownDAC(uint8_t DAC, uint8_t downUp){
	SPI_PORT &= ~(_BV(SPI_CS));
	
	uint8_t data1 = 0x04;	//C3=0 C2=1 C1=0 C0=0 Power-Up/Power-Down Function
	uint8_t data2 = 0x00;
	uint8_t data3 = 0x00;
	if(downUp==0)
		data3 = 0x02;		//100k do GND
	uint8_t data4 = 0x00;
	if (DAC == 0)
	data4 = 0x03;
	else
	data4 = 0x0C;
	
	spi_data_send(data1);
	spi_data_send(data2);
	spi_data_send(data3);
	spi_data_send(data4);
	
	SPI_PORT |= (_BV(SPI_CS));
}

void writeToDAC_withPowerDown(uint8_t DAC, uint16_t value){
	static uint8_t prevDAC[2]={0,0};
	if(value==0){
		powerDownDAC(DAC,0);		//power off
		prevDAC[DAC]=0;
	}		
	else{
		if(prevDAC[DAC]==0){
			powerDownDAC(DAC,1);	//power on
			prevDAC[DAC]=1;	
		}
		writeToDAC(DAC,value);
	}
}

long int getDACcode(double voltage){
	long int DACvalue = (long int)((double)((voltage*65536))/2.5);
	return DACvalue;
}

//----------------------RS-232--------------------------
void sendToUART(char data[80]){
	for (int i=0;i<80;i++){
		uart_putc(data[i]);
		if(data[i]=='\n')
		break;
	}
}

//----------------------INPUTS--------------------------
uint16_t readDataFrom74HC165(){
	uint16_t data=0;
	PORTD |= 0x20;	//LOAD=1
	
	for(uint8_t i=0;i<16;i++){
		PORTD &= ~0x10;		//CLK low
		if (!(PIND&0x08))	//DATA = 0 button is pressed
			data+=(1<<i);
		
		PORTD |= 0x10;		//CLK high
	}
	PORTD &= ~0x20;	//LOAD=0
	return data;
}

void writeDataTo74HC4094(uint8_t data){
	for (int i=7;i>=0;i--){
		PORTD |= 0x40;	//CLK up
		
		if(data & (1<<i))	//set DATA pin
			PORTD |= 0x80;
		else
			PORTD &= ~0x80;
		
		PORTD &= ~0x40;	//CLK down
	}
	PORTD |= 0x40;	//CLK up
}

//----------------------ADC--------------------------
#define PORT_ADC	PORTC
#define DDR_ADC		DDRC
#define PIN_ADC		PINC

#define SCL_ADC		0x20
#define SDA_ADC		0x01
#define DRDY_ADC	0x02
#define RFS_ADC		0x04
#define TFS_ADC		0x08
#define A0_ADC		0x10

#define CONT_REG	1
#define DATA_REG	0

/*#define SCL_ADC		0x01
#define SDA_ADC		0x02
#define DRDY_ADC	0x04
#define RFS_ADC		0x08
#define TFS_ADC		0x10
#define A0_ADC		0x20*/

void delay(){
	uint8_t y=2;
	for (uint8_t i=0; i<1;i++){
		y++;
	}
}

void writeToADC(unsigned long int data){

	DDR_ADC |= SDA_ADC;			//data output
	PORT_ADC|= SDA_ADC;
	delay();
	PORT_ADC &= ~SCL_ADC;		//clock up
	delay();
	PORT_ADC &= ~A0_ADC;
	delay();
	PORT_ADC &= ~TFS_ADC;
	delay();
	
	for (uint8_t i=24;i>0;i--){
		PORT_ADC &= ~SCL_ADC;		//clock down
		delay();
		
		if(((data>>(i-1)))&0x00001){
			PORT_ADC |= SDA_ADC;	
			//uart_putc('1');
		}			
		else {
			PORT_ADC &= ~SDA_ADC;	
			//uart_putc('0');
		}			
		
		PORT_ADC |= SCL_ADC;		//clock up
		delay();
	}
	
	PORT_ADC &= ~SCL_ADC;
	
	delay();
	PORT_ADC |= TFS_ADC;
	
	delay();
	PORT_ADC |= A0_ADC;
	
	delay();
	DDR_ADC &= ~SDA_ADC;			//data input
}

uint32_t readFromADC(uint8_t reg){
	uint32_t result=0;
	
	PORT_ADC |= SCL_ADC;		//clock up
	
	if((!(PIN_ADC&DRDY_ADC)) || reg==CONT_REG)	{	//DRDY pull low
		delay();
		if(reg==CONT_REG){
			PORT_ADC &= ~A0_ADC;
			delay();
		}			
		PORT_ADC &= ~RFS_ADC;
		delay();
		
		for (uint8_t i=0;i<24;i++){
			PORT_ADC &= ~SCL_ADC;		//clock down
			delay();
			if (PIN_ADC&SDA_ADC)		//read data
				result += (1<<i);
			PORT_ADC |= SCL_ADC;		//clock up
			delay();
		}
		
		delay();
		PORT_ADC |= RFS_ADC;
		
		delay();
		PORT_ADC |= A0_ADC;
	}
	else
		result=0xFFFFFFFF;
	
	return result;
}

void init(){
	//-----------------SPI----------------------
	spi_init();
	
	//----------------------74HC165--------------------
	DDRD &= ~0x08;	//Data out from inputs
	DDRD |=  0x30;	//CLK & Load outputs
	PORTD |= 0x10;  //CLK high
	
	//---------------------74HC4094---------------------
	DDRD |= 0xC0;	//DATA & CLK as output
	PORTD|= 0xC0;	//DATA & CLK high
	
	//------------------------ADC-----------------------
	DDR_ADC  |= (SCL_ADC+RFS_ADC+TFS_ADC+A0_ADC);			//output
	PORT_ADC |= (SCL_ADC+RFS_ADC+TFS_ADC+A0_ADC);			//RFS & TFS & A0 high
	//PORT_ADC |= DRDY_ADC;									//DRDY pull up
	
	//-----------------------Relays---------------------
	DDRB |= 0x03;	//output PB0 & PB1
	PORTB &= ~0x03;	//PB0 & PB1 = 0
	
	//-----------------------RS-232---------------------
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));
	sei();
	
}

unsigned char reverse(unsigned char b) {
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

uint8_t relayOn=0;		//0 -> 10k		1-> 0.01		2-> 20

void constantPowerMode(long int setPower_uW){
	
}	

void constantVoltageMode(long int setVoltage_uV){
	writeToDAC_withPowerDown(1,setVoltage_uV);
}

void constantCurrentMode(long int setCurrent_uA){
	static long int prevCurrent_uA=0;
	static long int DACvalue=0;
	long int DACvalue_buff = 0;
	if(setCurrent_uA>=30000){
		if(prevCurrent_uA<30000){
			PORTB |=  0x01;
			PORTB &= ~0x02;
			relayOn=1;
		}			
		DACvalue_buff = getDACcode((double)(((double)setCurrent_uA/1e6)/100));	//U=I*R R=0.01 => U=I/100		*20
	}
	else if(setCurrent_uA<30000){
		if(prevCurrent_uA>=30000){
			PORTB &= ~0x01;
			PORTB |=  0x02;
			relayOn=2;
		}
		DACvalue_buff = getDACcode((double)(((double)setCurrent_uA/1e6)*20));
	}
	if(DACvalue_buff!=DACvalue){
		prevCurrent_uA=setCurrent_uA;
		DACvalue=DACvalue_buff;
		writeToDAC_withPowerDown(1,DACvalue);
		
		char buff[20];
		sprintf(buff,"DAC value = %ld\n",DACvalue);
		sendToUART(buff);
	}
}

#define KEYPRESS_TIME	200

int main(void) {
	
	init();
	
	unsigned long int control_register=0xA081FF;//0xA083FFC;//0xA081FF;
	
	writeToADC(control_register);
	uint32_t ADC_control_reg = readFromADC(CONT_REG);
	
	//test to read ADC control reg
	char buff2[20];
	sprintf(buff2,"CTR: %X %X %X\n",reverse((uint8_t)(ADC_control_reg)),reverse((uint8_t)(ADC_control_reg>>8)),reverse((uint8_t)(ADC_control_reg>>16)));
	sendToUART(buff2);
	
	_delay_ms(100);
	
	long int printValue=0;
	long int setValue=0;
	
	/*mode = 3 constant current
	  mode = 2 constant voltage
	  mode = 1 constant resistance
	  mode = 0 constant power
	  
	  mode = 10 measure voltage
	  mode = 11 measure current
	*/
	uint8_t mode = 3;
	uint8_t newMode=0;
	uint16_t blinkingMode = 0;
	
	uint8_t changeMode=0;
	int selectDigit=-1;
	
	writeDataTo74HC4094(1<<mode);
	uint16_t TimeCounter=0;
	uint16_t keyPress=0;
	
	uint32_t currentVoltage=0;
	
	powerDownDAC(0,0);
	
    while(1) {
		TimeCounter++;
		if(TimeCounter==1000)
			TimeCounter=0;
		if(blinkingMode){
			blinkingMode++;
			if(blinkingMode==600){
				blinkingMode=1;
				writeDataTo74HC4094(0x00);
			}				
			if(blinkingMode==300){
				if(changeMode)
					writeDataTo74HC4094(1<<newMode);
				else
					writeDataTo74HC4094(1<<mode);
			}				
		}
		
		/*Buttons*/
		uint16_t buttons=readDataFrom74HC165();
		if(keyPress==0){
			if(changeMode){
				if(buttons&0x0008){	//OK  button
					setValue=printValue;
					mode=newMode;
					writeDataTo74HC4094(1<<mode);
					changeMode=0;
					blinkingMode=0;
					selectDigit=-1;
					keyPress=KEYPRESS_TIME;
					//write to DAC and change something
				}
				else if(buttons&0x0004){//CANCEL button
					printValue=setValue;
					blinkingMode=0;
					writeDataTo74HC4094(1<<mode);
					changeMode=0;
					selectDigit=-1;
					keyPress=KEYPRESS_TIME;
					//do something?? probable not
				}
				else if(buttons&0x0080){	//^ button
					switch (selectDigit){
						case 0:
						printValue++; break;
						case 1:
						printValue+=10; break;
						case 2:
						printValue+=100; break;
						case 3:
						printValue+=1000L; break;
						case 4:
						printValue+=10000L; break;
						case 5:
						printValue+=100000L; break;
						case 6:
						printValue+=1000000L; break;
					}
					if(printValue>10000000)
						printValue=10000000;
					keyPress=KEYPRESS_TIME;
				}
				else if(buttons&0x0010){	//V button
					switch (selectDigit){
						case 0:
							printValue--; break;
						case 1:
							printValue-=10; break;
						case 2:
							printValue-=100; break;
						case 3:
							printValue-=1000L; break;
						case 4:
							printValue-=10000L; break;
						case 5:
							printValue-=100000L; break;
						case 6:
							printValue-=1000000L; break;
					}
					if(printValue<0)
						printValue=0;
					keyPress=KEYPRESS_TIME;
				}
				else if(buttons&0x0040){	//< button
					if(selectDigit<6)
						selectDigit++;
					keyPress=KEYPRESS_TIME;
				}
				else if(buttons&0x0020){	//> button
					if(selectDigit!=0)
						selectDigit--;
					keyPress=KEYPRESS_TIME;
				}
			}
			else if((buttons&0x0008)){
				newMode=mode;
				blinkingMode=1;
				printValue=setValue;
				changeMode=1;
				if(mode==2)
					selectDigit=3;
				if(mode==3)
					selectDigit=5;
				keyPress=KEYPRESS_TIME;
			}
		
			/*mode buttons*/
			if(buttons&0x1000){	//I
				printValue=setValue;
				newMode=3;
				blinkingMode=1;
				changeMode=1;
				selectDigit=0;
				keyPress=KEYPRESS_TIME;
			}
			if(buttons&0x2000){	//U
				printValue=setValue;
				newMode=2;
				blinkingMode=1;
				changeMode=1;
				selectDigit=0;
				keyPress=KEYPRESS_TIME;
			}
			if(buttons&0x4000){	//R
				printValue=setValue;
				newMode=1;
				blinkingMode=1;
				changeMode=1;
				selectDigit=0;
				keyPress=KEYPRESS_TIME;
			}
			if(buttons&0x8000){	//P
				printValue=setValue;
				newMode=0;
				blinkingMode=1;
				changeMode=1;
				selectDigit=0;
				keyPress=KEYPRESS_TIME;
			}
		}
		else
			keyPress--;	
		_delay_ms(1);
			
		/*ADC*/
		uint32_t dataFromADC=readFromADC(DATA_REG);
		static long long ADC1_Voltage=0;
		static long long ADC2_Voltage=0;
		static uint8_t channelCounter=0;
		if(dataFromADC!=0xFFFFFFFF){
			currentVoltage = (uint32_t)reverse((uint8_t)dataFromADC)*256*256+(uint32_t)reverse((uint8_t)(dataFromADC>>8))*256+(uint32_t)reverse((uint8_t)(dataFromADC>>16));
			if(control_register==0xA281FF){	//ADC2 choosed
				ADC2_Voltage=((long long)currentVoltage-(long long)8388608)*(long long)2500000*16/(long long)8388608;
				if(channelCounter==10){
					control_register=0xA081FF;
					writeToADC(control_register);
					channelCounter=0;
				}				
			}
			else{							//ADC1 choosed
				ADC1_Voltage=((long long)currentVoltage-(long long)8388608)*(long long)2500000/(long long)8388608;
				if(channelCounter==10){
					control_register=0xA281FF;
					writeToADC(control_register);
					channelCounter=0;
				}					
			}
			channelCounter++;			
		}		
		
		/*UART print*/
		if(TimeCounter%500==0){
			char buff[80];
			if(changeMode)
				sprintf(buff,"%ld.%3ldmA nC:%d\t",printValue/1000,printValue%1000,selectDigit);	
			else
				sprintf(buff,"%ld.%3ldmA\t",printValue/1000,printValue%1000);
			//long int I_device = 0;
			switch (relayOn){
				case 0:
					//I_device=ADC1_Voltage/10000;
					sprintf(buff,"%s ADC1: %ld uV \t ADC2: %ld uV \t I: %ld uA\n",buff, (long int)ADC1_Voltage,(long int)ADC2_Voltage,(long int)ADC1_Voltage/500000);
					break;
				case 1:
					//I_device=ADC1_Voltage*100;
					sprintf(buff,"%s %ld uV\t%ld uV\tI: %ld uA\n",buff, (long int)ADC1_Voltage,(long int)ADC2_Voltage,(long int)ADC1_Voltage*2);
					break;
				case 2:
					//I_device=ADC1_Voltage/20;
					sprintf(buff,"%s ADC1: %ld uV \t ADC2: %ld uV \t I: %ld uA\n",buff, (long int)ADC1_Voltage,(long int)ADC2_Voltage,(long int)ADC1_Voltage/1000);
					break;
			}
			
			sendToUART(buff);
		}		
		
		/*Mode*/
		switch (mode){
			case 0:	//const P 
				switch (relayOn){
					case 0:
						//constantPowerMode(setValue, (long int)ADC1_Voltage/100000);
						break;
					case 1:
						//constantPowerMode(setValue, (long int)ADC1_Voltage*10);
						break;
					case 2:
						//constantPowerMode(setValue, (long int)ADC1_Voltage/200);
						break;
				}
				constantPowerMode(setValue);
				break;
			case 1: //const R
				break;
			case 2: //const U
				constantVoltageMode(setValue);
				break;
			case 3: //const I
				constantCurrentMode(setValue);
				break;
		}
		
    }
}