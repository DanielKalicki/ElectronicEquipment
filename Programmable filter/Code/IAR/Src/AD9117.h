#ifndef __AD9117_H
#define __AD9117_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32F4xx_nucleo.h"

//#include "mbed.h"
  
        void wait_ms(uint8_t value);

        void AD9117_sendByte(uint8_t value);
        uint8_t AD9117_readByte();
        
        #define  SPI_CONTROL_REG    0x00
        #define  POWER_DOWN_REG     0x01
        #define  DATA_CONTROL_REG   0x02
        #define  I_DAC_GAIN_REG     0x03
        #define  IRSET_REG          0x04
        #define  IRCML_REG          0x05
        #define  Q_DAC_GAIN_REG     0x06
        #define  QRSET_REG          0x07
        #define  QRCML_REG          0x08
        #define  AUXDAC_Q_REG       0x09
        #define  AUX_CTLQ_REG       0x0A
        #define  AUXDAC_I_REG       0x0B
        #define  AUX_CTLI_REG       0x0C
        #define  REFERENCE_RESISTOR_REG 0x0D
        #define  CAL_CONTROL_REG    0x0E
        #define  CAL_MEMORY_REG     0x0F
        #define  MEMORY_ADDRESS_REG 0x10
        #define  MEMORY_DATA_REG    0x11
        #define  MEMORY_RW_REG      0x12
        #define  CLKMODE_REG        0x14
        #define  Version_REG        0x1F
        
        
    
        
        void AD9117_init();
        void AD9117_output(int);
        uint8_t AD9117_readRegister(uint8_t number);
        void AD9117_setRegister(uint8_t number, uint8_t value);
        
        #define LSBFIRST        0x40
        #define AD9117_RESET    0x20
        #define LNGINS          0x10
        void AD9117_setSpiInterface(int8_t LSB_first, int8_t RESET, int8_t bitAddress13);
        
        #define LDOOFF      0x80
        #define LDOSTAT     0x40
        #define PWRDN_Q     0x20
        #define DACOFF      0x10
        #define I_DACOFF    0x08
        #define QCLKOFF     0x04
        #define ICLKOFF     0x02
        #define EXTREF      0x01
        void AD9117_setPowerDown(int8_t LDO_off, int8_t LDO_status, int8_t PowerDown_AD, int8_t QDAC_off, int8_t IDAC_off, int8_t QClock_off, int8_t IClock_off,int8_t ExternalRef_on);
        
        #define TWOS        0x80
        #define IFIRST      0x20
        #define IRISING     0x10
        #define SIMULBIT    0x08
        #define DCI_EN      0x04
        #define DCOSGL      0x02
        #define DCODBL      0x01
        void AD9117_setDataControl(int8_t two_compl, int8_t I_first_data_pads, int8_t I_rising_edge, int8_t dis_simult_inp_and_output_on_DCLKIO, int8_t data_clock_input_disabled, int8_t data_clock_output_enabled, int8_t DCODBL_enabled);
        
        #define IDACGAIN 0x3F;
        void AD9117_setIDacGain(uint8_t value);
        
        #define QDACGAIN 0x3F;
        void AD9117_setQDacGain(uint8_t value);
    
        #define IRSETEN      0x80
        #define IRSET_MASK   0x3F
        void AD9117_setIRSET(int8_t external_resistor, int8_t internal_resistor_value);
        
        #define QRSETEN     0x80
        #define QRSET_MASK   0x3F
        void AD9117_setQRSET(int8_t external_resistor, int8_t internal_resistor_value);

        #define IRCMLEN 0x80
        #define IRCML_MASK   0x3F
        void AD9117_setIRCML(int8_t external_resistor, int8_t internal_resistor_value);
        
        #define QRCMLEN 0x80
        #define QRCML_MASK   0x3F
        void AD9117_setQRCML(int8_t external_resistor, int8_t internal_resistor_value);

        #define QAUXDAC_MASK   0xFF
        void AD9117_setAuxDacQ(uint8_t value);
        
        #define QAUXEN  0x80
        #define QAUXRNG 0x60
        #define QAUXOFS 0x1C
        #define QAUXDAC_ADJ 0x03
        void AD9117_setAuxCtlQ(int8_t QAuxEnabled, int8_t QAuxOutputVoltage, int8_t QAuxTopVoltage, int8_t QAuxVoltAdj);
        
        #define IAUXDAC_MASK   0xFF
        void AD9117_setAuxDacI(uint8_t value);
        
        #define IAUXEN  0x80
        #define IAUXRNG 0x60
        #define IAUXOFS 0x1C
        #define IAUXDAC_ADJ 0x03
        void AD9117_setAuxCtlI(int8_t IAuxEnabled, int8_t IAuxOutputVoltage, int8_t IAuxTopVoltage, int8_t IAuxVoltAdj);
        
        #define RREF   0x3F
        void AD9117_setRefResistor(int8_t value);
        
        #define PRELDQ 0x80
        #define PRELDI 0x40
        #define CALSELQ 0x20
        #define CALSELI 0x10
        #define CALCLK 0x08
        #define DIVSEL_MASK 0x07
        void AD9117_setCalControl (int8_t QDacCalibSetByUser, int8_t IDacCalibSetByUser, int8_t QDacSelfCalib, int8_t IDacSelfCalib, int8_t CalibClockEnabled, int8_t ClockDivValue);
        
        #define CALSTA 0x80
        #define CALSTATI 0x40
        #define CALMEMQ 0x0C;
        #define CALMEMI 0x03
        //void setCalMemory(int8_t 
        
        #define MEMADDR_MASK 0x3F
        void AD9117_setMemoryAddress(int8_t address);
        
        #define MEMDATA_MASK 0x3F;
        void AD9117_setMemoryData(int8_t data);

        #define CALRSTQ 0x80
        #define CALRSTI 0x40
        #define CALEN   0x10
        #define SMEMWR  0x08
        #define SMEMRD  0x04
        #define UNCALQ  0x02
        #define UNCALI  0x01
        void AD9117_setMemoryRW(int8_t clearCALSTATQ, int8_t clearCALSTATI, int8_t init_self_cal, int8_t write_to_static_memory, int8_t read_from_static_memory, int8_t reset_QDac_callib, int8_t reset_IDac_callib);
        
        #define CLKMODEQ_MASK 0xC0
        #define Searching_MASK 0x10
        #define Reacquire_MASK 0x08
        #define CLKMODEN_MASK 0x04
        #define CLKMODEI_MASK 0x03
        void AD9117_setCLKMODE(int8_t CLKMODEQ_value, int8_t search_indic, int8_t Reacaquire_indic, int8_t CLKMODEN_value, int8_t CLKMODEI_value);




#endif /* __AD9117_H */