/*
 * ElectronicLoad.c
 *
 * Created: 2014-03-23 11:15:36
 *  Author: Daniel
 */ 

#ifndef F_CPU
#define F_CPU	16000000UL
#endif

//#define UART_BAUD_RATE      56000


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
//#include "uart.h"
#include "i2cmaster.h"
#include <stdio.h>

uint8_t relayOn=0;		//0 -> 100		1-> 0.01		2-> 1

//--------------------------LCD-------------------------
void writeDataTo74HC595(uint8_t E,uint8_t RS, uint8_t D4, uint8_t D5, uint8_t D6, uint8_t D7){
	for (int i=7;i>=0;i--){
		PORTA &= ~0x02;	//Shift clock down
		
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
			PORTA |= 0x08;
		else
			PORTA &= ~0x08;
		
		PORTA |= 0x02;	//shift clock up => Shift the register
	}
	PORTA &= ~0x02;	//shift clock down
	
	PORTA &= ~0x04;	//strobe clock down;
	PORTA |= 0x04;	//strobe clock up    =>  output data from register on the pins outputs
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

uint16_t DAC_code = 0;

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
	
	DAC_code=value;
	
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
	
	if(downUp==0)
		DAC_code=0;
	
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
	double multCoefficient=20;
	if(relayOn==1)
		multCoefficient=21.5;
	else if(relayOn==2)
		multCoefficient=21.1;
	long int DACvalue = (long int)((double)(multCoefficient*(voltage*65536))/2.5);
	return DACvalue;
}

//----------------------RS-232--------------------------
/*void sendToUART(char data[80]){
	for (int i=0;i<80;i++){
		uart_putc(data[i]);
		if(data[i]=='\n')
		break;
	}
}*/

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

#define SCL_ADC		0x01
#define SDA_ADC		0x02
#define DRDY_ADC	0x04
#define RFS_ADC		0x08
#define TFS_ADC		0x10
#define A0_ADC		0x20

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
	//------------------------LCD----------------------
	DDRA |= 0x0E;			//PA7 - PA5 as outputs
	PORTA &= ~0x0E;			//PA7 - PA5 set to low
	
	//------------------------SPI----------------------
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
	//uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));
	sei();
	
	initLCD();
	
}

unsigned char reverse(unsigned char b) {
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

long int device_current(long int ADC1){
	switch (relayOn){
		case 0:
			return (ADC1/5050);
			break;
		case 1:
			return (ADC1*200/101);
			break;
		case 2:
			return (ADC1*2/101);
		break;
	}
	return 0;
}

uint8_t newADC1=0;

void constantCurrentMode(long int setCurrent_uA,long int ADC1_Voltage){
	static long int prevCurrent_uA=0;
	static long int DACvalue=0;
	long int DACvalue_buff = 0;
	if(setCurrent_uA>=30000){
		if(prevCurrent_uA<30000 || relayOn!=1){
			PORTB |=  0x01;
			PORTB &= ~0x02;
			relayOn=1;
		}			
		DACvalue_buff = getDACcode((double)(((double)setCurrent_uA/1e6)/100));	//U=I*R R=0.01 => U=I/100		*20
	}
	else if(setCurrent_uA<30000){
		if(prevCurrent_uA>=30000 || relayOn!=2){
			PORTB &= ~0x01;
			PORTB |=  0x02;
			relayOn=2;
		}
		DACvalue_buff = getDACcode((double)(((double)setCurrent_uA/1e6)));
	}
	if(DACvalue_buff!=DACvalue){
		prevCurrent_uA=setCurrent_uA;
		DACvalue=DACvalue_buff;
		writeToDAC_withPowerDown(1,DACvalue);
		
		//char buff[20];
		//sprintf(buff,"DAC value = %ld\n",DACvalue);
		//sendToUART(buff);
	}
	/*else {
		if(setCurrent_uA>device_current(ADC1)){		//increase DAC
			if(DAC_code<26000 && newADC1==1){
				DAC_code++;
				newADC1=0;
				writeToDAC_withPowerDown(1,DAC_code);
			}				
		}
		else if(setCurrent_uA<device_current(ADC1)){
			if(DAC_code != 0 && newADC1==1){
				DAC_code--;
				newADC1=0;
				writeToDAC_withPowerDown(1,DAC_code);
			}				
		}
	}*/
}

void constantPowerMode(long int setPower_uW, long int ADC1_Voltage, long int ADC2_Voltage){
	constantCurrentMode((long int)(((double)setPower_uW/(double)ADC2_Voltage)*1000000),ADC1_Voltage);
}

void constantResistanceMode(long int setResistance_mOhm, long int ADC1_Voltage, long int ADC2_Voltage){
	constantCurrentMode((long int)((double)ADC2_Voltage/((double)setResistance_mOhm*1000)*1000000),ADC1_Voltage);
}

void constantVoltageMode(long int setVoltage_uV){
	static long int prevVoltage_uV = 0;
	if(prevVoltage_uV!=setVoltage_uV){
		long int DACvalue_buff = getDACcode((double) (setVoltage_uV/1000000.0));
		writeToDAC_withPowerDown(1,DACvalue_buff);
		prevVoltage_uV=setVoltage_uV;
	}	
}

#define KEYPRESS_TIME	2000
#define BLINKING_TIME	5000
#define SCREEN_TIME		500

int main(void) {
	
	init();
	
	unsigned long int control_register=0xA081FF;//0xA083FFC;//0xA081FF;
	
	writeToADC(control_register);
	uint32_t ADC_control_reg = readFromADC(CONT_REG);
	
	//test to read ADC control reg
	//char buff2[20];
	//sprintf(buff2,"CTR: %X %X %X\n",reverse((uint8_t)(ADC_control_reg)),reverse((uint8_t)(ADC_control_reg>>8)),reverse((uint8_t)(ADC_control_reg>>16)));
	//sendToUART(buff2);
	
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
	uint8_t newMode=3;
	uint16_t blinkingMode = 0;
	
	uint8_t changeMode=0;
	int selectDigit=-1;
	
	writeDataTo74HC4094(1<<mode);
	uint16_t TimeCounter=0;
	uint16_t keyPress=0;
	
	uint8_t select_ADC_Channel=1;
	
	uint32_t currentVoltage=0;
	
	uint8_t ADC2_Gain=0;
	
	powerDownDAC(0,0);
	
    while(1) {
		/*-------------------------------------------------------------Time----------------------------------------------------------------*/
		TimeCounter++;
		if(TimeCounter==60000)
			TimeCounter=0;
		if(blinkingMode){
			blinkingMode++;
			if(blinkingMode==BLINKING_TIME){
				blinkingMode=1;
				writeDataTo74HC4094(0x00);
			}				
			if(blinkingMode==BLINKING_TIME/2){
				if(changeMode){
					if(newMode<10)	
						writeDataTo74HC4094(1<<newMode);	//blinking diode over new mode button
					else if(newMode==11)
						writeDataTo74HC4094(0x03);	//on read Voltage mode buttons on I&U mode blinks
					else
						writeDataTo74HC4094(0x0C);	//on read Current mode buttons on R&P mode blinks
				}					
				else
					writeDataTo74HC4094(1<<mode);
			}				
		}
		
		/*-------------------------------------------------------------Buttons---------------------------------------------------------------*/
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
					newMode=mode;
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
			else if((buttons&0x0008)){		//OK button pressed
				newMode=mode;
				blinkingMode=1;
				printValue=setValue;
				changeMode=1;
				if(mode==2)
					selectDigit=3;
				if(mode==3)
					selectDigit=5;
				else 
					selectDigit=1;
				keyPress=KEYPRESS_TIME;
				writeDataTo74HC4094(0x00);
			}
		
			/*mode buttons*/
			if(buttons&0x0002){	// Voltage read mode
				printValue=setValue;
				newMode=10;
				changeMode=1;
				blinkingMode=1;
				keyPress=KEYPRESS_TIME;
				writeDataTo74HC4094(0x00);
			}
			if(buttons&0x0001){	//current read mode (dangerous)
				printValue=setValue;
				newMode=11;
				changeMode=1;
				blinkingMode=1;
				keyPress=KEYPRESS_TIME;
				writeDataTo74HC4094(0x00);
			}
			if(buttons&0x1000){	//I
				printValue=setValue;
				newMode=3;
				blinkingMode=1;
				changeMode=1;
				selectDigit=0;
				keyPress=KEYPRESS_TIME;
				writeDataTo74HC4094(0x00);
			}
			if(buttons&0x2000){	//U
				printValue=setValue;
				newMode=2;
				blinkingMode=1;
				changeMode=1;
				selectDigit=0;
				keyPress=KEYPRESS_TIME;
				writeDataTo74HC4094(0x00);
			}
			if(buttons&0x4000){	//R
				printValue=setValue;
				newMode=1;
				blinkingMode=1;
				changeMode=1;
				selectDigit=0;
				keyPress=KEYPRESS_TIME;
				writeDataTo74HC4094(0x00);
			}
			if(buttons&0x8000){	//P
				printValue=setValue;
				newMode=0;
				blinkingMode=1;
				changeMode=1;
				selectDigit=0;
				keyPress=KEYPRESS_TIME;
				writeDataTo74HC4094(0x00);
			}
		}
		else
			keyPress--;	
		//_delay_ms(1);
			
		/*---------------------------------------------------------------ADC---------------------------------------------------------------*/
		uint32_t dataFromADC=readFromADC(DATA_REG);
		static long long ADC1_Voltage=0;
		static long long ADC2_Voltage=0;
		static uint8_t channelCounter=0;
		static uint16_t FFcounter=0;
		if(dataFromADC!=0xFFFFFFFF){
			currentVoltage = (uint32_t)reverse((uint8_t)dataFromADC)*256*256+(uint32_t)reverse((uint8_t)(dataFromADC>>8))*256+(uint32_t)reverse((uint8_t)(dataFromADC>>16));
			//if(control_register==0xA281FF){	//ADC2 choosed
			
			if(mode==10){	//read V mode
				if(select_ADC_Channel==1){	//change to channel ADC2
					control_register=0xA281FF;
					select_ADC_Channel=2;
					writeToADC(control_register);
				}
				channelCounter=0;	//it prevent switching channels
				/*if(control_register!=0xA281FF){
					control_register=0xA281FF;
					writeToADC(control_register);
				}					*/
			}				
				
			if(select_ADC_Channel==2){
				if(mode!=10)
					ADC2_Voltage=((long long)currentVoltage-(long long)8388608)*(long long)2500000*16/(long long)8388608;
				else{			//read voltage mode
					//if we try to read precisely the input voltage than dont multiply the result by 16 yet
					ADC2_Voltage=((long long)currentVoltage-(long long)8388608)*(long long)2500000/(long long)8388608;
					//ADC2_Voltage=(long long)((double)currentVoltage-(double)8388608.0)*(double)2500000.0/(double)8388608.0;
					
					if(ADC2_Voltage<900000 && ADC2_Voltage>0){	//read voltage < 0.9V => that meanse we can increase ADC gain
						if(ADC2_Gain<7){
							ADC2_Gain++;
							control_register=0xA283EF+(uint32_t)ADC2_Gain*(uint32_t)0x40000;
							writeToADC(control_register);
							_delay_ms(10);
						}						
					}
					else if(ADC2_Voltage>2400000 || ADC2_Voltage<0){	//voltage > 2.4 that means we should decrease the gain
						if(ADC2_Gain!=0){
							ADC2_Gain--;
							control_register=0xA283EF+(uint32_t)ADC2_Gain*(uint32_t)0x40000;
							writeToADC(control_register);
							_delay_ms(10);
						}						
					}
				}		//end mode==10		
				if(channelCounter==10){			//mode < 10
					control_register=0xA081FF;
					select_ADC_Channel=1;
					writeToADC(control_register);
					channelCounter=0;
					ADC2_Gain=0;
				}				
			}
			else{							//ADC1 choosed
				ADC1_Voltage=((long long)currentVoltage-(long long)8388608)*(long long)2500000/(long long)8388608;
				newADC1=1;
				if(channelCounter==10){
					control_register=0xA281FF;	//0xA28013
					select_ADC_Channel=2;
					writeToADC(control_register);
					channelCounter=0;
				}					
			}
			channelCounter++;
			FFcounter=0;		
		}
		else{
			FFcounter++;
			if(FFcounter>=5000 && mode==10){		//if we didnt get a valid result in 1000 tryies it can mean that the gain is set to high and the ADC is overrange
				FFcounter=0;
				if(ADC2_Gain!=0){		//decrease gain
					ADC2_Gain--;
					control_register=0xA283EF+(uint32_t)ADC2_Gain*(uint32_t)0x40000;
					writeToADC(control_register);
					_delay_ms(10);
				}
			}
		}			
		
		/*----------------------------------------------------------LCD--------------------------------------------------------------*/
		if(TimeCounter%SCREEN_TIME==0){
			char buff[80];
			/*if(changeMode)
				sprintf(buff,"%ld.%3ldmA nC:%d\t",printValue/1000,printValue%1000,selectDigit);	
			else
				sprintf(buff,"%ld.%3ldmA\t",printValue/1000,printValue%1000);
			
			sprintf(buff,"%s A1:%lduV\tA2:%lduV\tI:%lduA\t%ud\n",buff, (long int)ADC1_Voltage,(long int)ADC2_Voltage,device_current(ADC1_Voltage),(unsigned int)DAC_code);
			sendToUART(buff);*/
			
			clearLCD();
			_delay_ms(2);
			uint8_t digitCounter=0;
			uint8_t dotPosition=7;
			if(newMode == 10 && changeMode==1){
				sprintf(buff,"Voltage mode\n");
				for (uint8_t i=0;i<=16;i++){	//prints the number on LCD
					if(buff[i]=='\n') break;
					writeToLCD_Letter(buff[i]);
				}
			}
			else if(newMode==11 && changeMode==1){
				sprintf(buff,"Current mode\n");
				for (uint8_t i=0;i<=16;i++){	//prints the number on LCD
					if(buff[i]=='\n') break;
					writeToLCD_Letter(buff[i]);
				}	
			}
			else {
				if(newMode < 10){		//constant value modes
					switch (newMode){
						case 0:
							writeToLCD_Letter('P'); break;
						case 1:
							writeToLCD_Letter('R'); break;
						case 2:
							writeToLCD_Letter('U'); break;
						case 3:
							writeToLCD_Letter('I'); break;
					}
					writeToLCD_Letter('s');
					writeToLCD_Letter('e');
					writeToLCD_Letter('t');
					writeToLCD_Letter(':');
					writeToLCD_Letter(' ');
					sprintf(buff,"%ld\n",printValue);
					if(newMode==1)	dotPosition=4;		//for om range
					for (uint8_t i=0;i<15;i++){			//counts the number of digit in set number
						if(buff[i]=='\n') break;
						else digitCounter++;
					}
					if(selectDigit>digitCounter-1 && newMode==1){		//Ads _ symbol indicating actualy changeable digit
						int digBuff=digitCounter-3;
						if(digBuff<0) digBuff=1;
						for (uint8_t i=0; i<selectDigit-2-digBuff;i++)
							writeToLCD_Letter('_');
					}				
					if (digitCounter<dotPosition){		//Ads extra zeros to fill the number to 0.000000 format
						if(selectDigit==dotPosition-1 && (TimeCounter%(SCREEN_TIME*4))<SCREEN_TIME) writeToLCD_Letter(' ');
						else writeToLCD_Letter('0');
						writeToLCD_Letter('.');
						for(uint8_t i=0;i<dotPosition-digitCounter-1;i++){
							if(dotPosition-selectDigit==i+2 && (TimeCounter%(SCREEN_TIME*4))<SCREEN_TIME) writeToLCD_Letter(' ');
							else writeToLCD_Letter('0');
						}
					}
					for (uint8_t i=0;i<=digitCounter;i++){	//prints the number on LCD
						if(buff[i]=='\n') break;
						if(selectDigit==digitCounter-i-1 && (TimeCounter%(SCREEN_TIME*4))<SCREEN_TIME) writeToLCD_Letter(' ');
						else writeToLCD_Letter(buff[i]);
						if(i==digitCounter-dotPosition)
							writeToLCD_Letter('.');
					}
					switch (newMode){
						case 0:
							writeToLCD_Letter('W'); break;
						case 1:
							writeToLCD_Letter(0xF4); break;		//0xF4 code for Omega
						case 2:
							writeToLCD_Letter('V'); break;
						case 3:
							writeToLCD_Letter('A'); break;
					}
					secondLineLCD();
					switch(newMode){
						case 0:
							writeToLCD_Letter('P');
							writeToLCD_Letter('a');
							writeToLCD_Letter('c');
							writeToLCD_Letter('t');
							break;
						case 1:
							writeToLCD_Letter('R');
							writeToLCD_Letter('a');
							writeToLCD_Letter('c');
							writeToLCD_Letter('t');
							break;
						case 2:
							writeToLCD_Letter('U');
							writeToLCD_Letter('i');
							writeToLCD_Letter('n');
							writeToLCD_Letter('p');
							break;
						case 3:
							writeToLCD_Letter('I');
							writeToLCD_Letter('a');
							writeToLCD_Letter('c');
							writeToLCD_Letter('t');
							break;
					}				
				}
				else if(mode==10 && changeMode==0){
					writeToLCD_Letter('V');
					writeToLCD_Letter('i');
					writeToLCD_Letter('n');
				}
				else if(mode==11 && changeMode==0){
					writeToLCD_Letter('I');
				}
				writeToLCD_Letter(':');
				writeToLCD_Letter(' ');
				if(ADC2_Voltage<0 && mode==10 && changeMode==0){
					writeToLCD_Letter('-');
					sprintf(buff,"%ld\n",(long int)((long int)(-1)*ADC2_Voltage));
				}
				else {
					if((mode==10 && changeMode==0) || (changeMode==1 && newMode==2 && mode==10)){
						switch(ADC2_Gain){			//im sorry
							case 0:
								sprintf(buff,"%ld\n",(long int)(ADC2_Voltage*16)); break;
							case 1:
								sprintf(buff,"%ld\n",(long int)(ADC2_Voltage*8)); break;
							case 2:
								sprintf(buff,"%ld\n",(long int)(ADC2_Voltage*4)); break;
							case 3:
								sprintf(buff,"%ld\n",(long int)(ADC2_Voltage*2)); break;
							case 4:
								sprintf(buff,"%ld\n",(long int)(ADC2_Voltage)); break;
							case 5:
								sprintf(buff,"%ld\n",(long int)(ADC2_Voltage/2)); break;
							case 6:
								sprintf(buff,"%ld\n",(long int)(ADC2_Voltage/4)); break;
							case 7:
								sprintf(buff,"%ld\n",(long int)(ADC2_Voltage/8)); break;
						}							
					}
					else if(newMode==3){	//show measured current
						sprintf(buff,"%ld\n",device_current(ADC1_Voltage));
					}
					else if(newMode==2 && (changeMode==0 || mode!=10)){
						sprintf(buff,"%ld\n",(long int)ADC2_Voltage);
					}
					else if(newMode==1){
						sprintf(buff,"%ld\n",(long int)((double)ADC2_Voltage/(double)device_current(ADC1_Voltage)*1000));
					}
					else if(newMode==0){	//show actual power
						sprintf(buff,"%ld\n",(long int)(device_current(ADC1_Voltage)/1000000.0*ADC2_Voltage));
					}
					else if(mode==11 && changeMode==0){
						sprintf(buff,"%d\n",DAC_code);
					}
				}
				digitCounter=0;
				for (uint8_t i=0;i<15;i++){
					if(buff[i]=='\n') break;
					else digitCounter++;
				}
				if (digitCounter<dotPosition){		//Ads extra zeros to fill the number to 0.000000 format
					writeToLCD_Letter('0');
					writeToLCD_Letter('.');
					for(uint8_t i=0;i<dotPosition-digitCounter-1;i++){
						writeToLCD_Letter('0');
					}
				}
				for (uint8_t i=0;i<=digitCounter;i++){
					if(buff[i]=='\n') break;
					 writeToLCD_Letter(buff[i]);
					if(i==digitCounter-dotPosition)
						writeToLCD_Letter('.');
				}
			
				if(newMode==0)										writeToLCD_Letter('W');
				if(newMode==1)										writeToLCD_Letter(0xF4);
				if((mode==10 && changeMode==0) || newMode==2)		writeToLCD_Letter('V');
				else if((mode==11 && changeMode==0) || newMode==3)	writeToLCD_Letter('A');
				
				//print select gain
				if(mode==10 && changeMode==0){
					secondLineLCD();
					sprintf(buff,"G: %d\n",ADC2_Gain);
					for (uint8_t i=0;i<8;i++){
						if(buff[i]=='\n') break;
						writeToLCD_Letter(buff[i]);
					}					
				}
			}				
		}		
		
		/*-------------------------------------------------------------Mode--------------------------------------------------------------*/
		switch (mode){
			case 0:	//const P
				constantPowerMode(setValue,ADC1_Voltage,ADC2_Voltage);
				break;
			case 1: //const R
				constantResistanceMode(setValue,ADC1_Voltage,ADC2_Voltage);
				break;
			case 2: //const U
				constantVoltageMode(setValue);
				break;
			case 3: //const I
				constantCurrentMode(setValue,ADC1_Voltage);
				break;
			case 10:	//read V
				constantCurrentMode(0,ADC1_Voltage);		//mosfet turn off
				break;
			case 11:	//read I
				break;
		}
		
    }
}