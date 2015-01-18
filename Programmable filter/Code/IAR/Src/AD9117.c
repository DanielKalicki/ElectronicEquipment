#include "AD9117.h"

//DigitalOut cs(D10); 
//DigitalOut mySCLK(D15);
//DigitalInOut mySDIO(D14);

struct AD9117_registers{
            uint8_t SPI_Control;
            uint8_t Power_Down;
            uint8_t Data_Control;
            uint8_t I_DAC_Gain;
            uint8_t IRSET;
            uint8_t IRCML;
            uint8_t Q_DAC_Gain;
            uint8_t QRSET;
            uint8_t QRCML;
            uint8_t AUXDAC_Q;
            uint8_t AUX_CTLQ;
            uint8_t AUXDAC_I;
            uint8_t AUX_CTLI;
            uint8_t Reference_Resistor;
            uint8_t Cal_Control;
            uint8_t Cal_Memory;
            uint8_t Memory_Address;
            uint8_t Memory_Data;
            uint8_t Memory_RW;
            uint8_t CLKMODE;
            uint8_t Version;
        } reg;

GPIO_InitTypeDef  GPIO_InitStruct_AD9117_SPI;
GPIO_InitTypeDef  GPIO_InitStruct_SDIO;
GPIO_InitTypeDef  GPIO_InitStruct_DAC;

#define AD9117_CS_PIN GPIO_PIN_8
#define AD9117_DCLKIO_PIN GPIO_PIN_9
#define AD9117_SCLK_PIN GPIO_PIN_2
#define AD9117_SDIO_PIN GPIO_PIN_12

void wait_ms(uint8_t value);
void AD9117_initGPIO();

void AD9117_setRegister(uint8_t number, uint8_t value){
    HAL_GPIO_WritePin(GPIOB,AD9117_CS_PIN,GPIO_PIN_RESET);
    wait_ms(1); 
    AD9117_sendByte(number);  
    AD9117_sendByte(value);  
    HAL_GPIO_WritePin(GPIOB,AD9117_CS_PIN,GPIO_PIN_SET);  
}

void AD9117_init(){
  
    AD9117_initGPIO();
  
    for (int i=0;i<22;i++){
        if (i==21) i=31;
        uint8_t regValue = AD9117_readRegister(i);
        switch (i){
            case SPI_CONTROL_REG:
                reg.SPI_Control=regValue; break;
            case POWER_DOWN_REG:
                reg.Power_Down=regValue;; break;
            case DATA_CONTROL_REG:
                reg.Data_Control=regValue;; break;
            case I_DAC_GAIN_REG:
                reg.I_DAC_Gain=regValue;; break;
            case IRSET_REG:
                reg.IRSET=regValue;; break;
            case IRCML_REG:
                reg.IRCML=regValue;; break;
            case Q_DAC_GAIN_REG:
                reg.Q_DAC_Gain=regValue;; break;
            case QRSET_REG:
                reg.QRSET=regValue;; break;
            case QRCML_REG:
                reg.QRCML=regValue;; break;
            case AUXDAC_Q_REG:
                reg.AUXDAC_Q=regValue;; break;
            case AUX_CTLQ_REG:
                reg.AUX_CTLQ=regValue;; break;
            case AUXDAC_I_REG:
                reg.AUXDAC_I=regValue;; break;
            case AUX_CTLI_REG:
                reg.AUX_CTLI=regValue;; break;
            case REFERENCE_RESISTOR_REG:
                reg.Reference_Resistor=regValue;; break;
            case CAL_CONTROL_REG:
                reg.Cal_Control=regValue;; break;
            case CAL_MEMORY_REG:
                reg.Cal_Memory=regValue;; break;
            case MEMORY_ADDRESS_REG:
                reg.Memory_Address=regValue;; break;
            case MEMORY_DATA_REG:
                reg.Memory_Data=regValue;; break;
            case MEMORY_RW_REG:
                reg.Memory_RW=regValue;; break;
            case CLKMODE_REG:
                reg.CLKMODE=regValue;; break;
            case Version_REG:
                reg.Version=regValue;; break;
            default:
                break;
        }
    }
    //setPowerDown(0,1,0,1,0,1,0,0);
    /*setRegister(0x01,0x54);
    setRegister(0x02,0x34);*/
    
    AD9117_setPowerDown(0,1,0,1,0,1,0,0);
    AD9117_setDataControl(0,1,1,0,1,0,0);
    wait_ms(10);
    //AD9117_setCalControl(0,0,0,1,1,0);
    //wait_ms(10);
    AD9117_setIRCML(1,0x1E);    //0x10
    wait_ms(10);
    AD9117_setIRSET(1,0x00);    //0x10
    wait_ms(10);
    AD9117_setIDacGain(0x00);//deosnt affect the dc offset
    wait_ms(10);
}

void AD9117_setSDIOasOutput(){
  GPIO_InitStruct_SDIO.Pin = GPIO_PIN_12;
  GPIO_InitStruct_SDIO.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_SDIO.Pull = GPIO_NOPULL;
  GPIO_InitStruct_SDIO.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_SDIO); 
}

void AD9117_setSDIOasInput(){
  GPIO_InitStruct_SDIO.Pin = GPIO_PIN_12;
  GPIO_InitStruct_SDIO.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct_SDIO.Pull = GPIO_NOPULL;
  GPIO_InitStruct_SDIO.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_SDIO); 
}

void AD9117_initGPIO(){
  //DAC pins
  GPIO_InitStruct_DAC.Pin = 0xFFFF;
  GPIO_InitStruct_DAC.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_DAC.Pull = GPIO_NOPULL;
  GPIO_InitStruct_DAC.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_DAC); 
  
  //communication pins
  GPIO_InitStruct_AD9117_SPI.Pin = GPIO_PIN_8 | GPIO_PIN_2 | AD9117_DCLKIO_PIN;
  GPIO_InitStruct_AD9117_SPI.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_AD9117_SPI.Pull = GPIO_NOPULL;
  GPIO_InitStruct_AD9117_SPI.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_AD9117_SPI); 
  
  AD9117_setSDIOasOutput();
}

void wait_ms(uint8_t value){
  for (int i=0;i<1000;i++){
    __asm("NOP");
  }
}

void AD9117_sendByte( uint8_t data){
    AD9117_setSDIOasOutput();
    
    if((data & (1<<8))>0){
      HAL_GPIO_WritePin(GPIOB,AD9117_SDIO_PIN,GPIO_PIN_SET);
      //mySDIO = 1;
    }
    else {
      HAL_GPIO_WritePin(GPIOB,AD9117_SDIO_PIN,GPIO_PIN_RESET);
        //mySDIO=0;
    }
    wait_ms(1);
    for (int i=7;i>-1;i--){
        HAL_GPIO_WritePin(GPIOB,AD9117_SCLK_PIN,GPIO_PIN_SET);//mySCLK=1;
        wait_ms(1);
        if((data & (1<<i))>0){
            HAL_GPIO_WritePin(GPIOB,AD9117_SDIO_PIN,GPIO_PIN_SET);//mySDIO = 1;
        }
        else {
            HAL_GPIO_WritePin(GPIOB,AD9117_SDIO_PIN,GPIO_PIN_RESET);//mySDIO=0;
        }
        HAL_GPIO_WritePin(GPIOB,AD9117_SCLK_PIN,GPIO_PIN_RESET);//mySCLK = 0;
        wait_ms(1);
    }
    HAL_GPIO_WritePin(GPIOB,AD9117_SCLK_PIN,GPIO_PIN_SET);//mySCLK=1;
    wait_ms(1);
}

uint8_t AD9117_readByte(){
    uint8_t value=0;
    AD9117_setSDIOasInput();
    for (int i=7;i>-1;i--){
        HAL_GPIO_WritePin(GPIOB,AD9117_SCLK_PIN,GPIO_PIN_RESET);//mySCLK = 0;
        wait_ms(1);
    
        if (HAL_GPIO_ReadPin(GPIOB,AD9117_SDIO_PIN)/*mySDIO.read()*/ == GPIO_PIN_SET){
            value += (1<<i);
        }

        HAL_GPIO_WritePin(GPIOB,AD9117_SCLK_PIN,GPIO_PIN_SET);//mySCLK=1;
        wait_ms(1);
    }
    HAL_GPIO_WritePin(GPIOB,AD9117_SCLK_PIN,GPIO_PIN_SET);//mySCLK=1;
    wait_ms(1);
    
    return value;
}

uint8_t AD9117_readRegister(uint8_t number){
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
    wait_ms(1); 
    AD9117_sendByte(0x80+number);  
    uint8_t ret = AD9117_readByte();
    wait_ms(1);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
    return ret;
}
//this function takes 13 clock cylces
void AD9117_output(int value){
  GPIOB->BSRRL = AD9117_DCLKIO_PIN;
  GPIOA->BSRRL = value;
  GPIOA->BSRRH = ~value;
  GPIOB->BSRRH = AD9117_DCLKIO_PIN;
}

void AD9117_setSpiInterface(int8_t LSB_first, int8_t RESET, int8_t bitAddress13){
    if      (LSB_first==0)      { reg.SPI_Control &= ~LSBFIRST; }
    else if (LSB_first==1)      { reg.SPI_Control |=  LSBFIRST; }
    
    if      (RESET==0)          { reg.SPI_Control &= ~AD9117_RESET; }
    else if (RESET==1)          { reg.SPI_Control |=  AD9117_RESET; }
    
    if      (bitAddress13==0)   { reg.SPI_Control &= ~LNGINS; }
    else if (bitAddress13==1)   { reg.SPI_Control |=  LNGINS; }
    
    AD9117_setRegister(SPI_CONTROL_REG,reg.SPI_Control);
}

void AD9117_setPowerDown(int8_t LDO_off, int8_t LDO_status, int8_t PowerDown_AD, int8_t QDAC_off, int8_t IDAC_off, int8_t QClock_off, int8_t IClock_off,int8_t ExternalRef_on){
    if      (LDO_off==0)                { reg.Power_Down &= ~LDOOFF; }
    else if (LDO_off==1)                { reg.Power_Down |=  LDOOFF; }
    
    if      (LDO_status==0)             { reg.Power_Down &= ~LDOSTAT; }
    else if (LDO_status==1)             { reg.Power_Down |=  LDOSTAT; }
    
    if      (PowerDown_AD==0)           { reg.Power_Down &= ~PWRDN_Q; }
    else if (PowerDown_AD==1)           { reg.Power_Down |=  PWRDN_Q; }
    
    if      (QDAC_off==0)               { reg.Power_Down &= ~DACOFF; }
    else if (QDAC_off==1)               { reg.Power_Down |=  DACOFF; }
    
    if      (IDAC_off==0)               { reg.Power_Down &= ~I_DACOFF; }
    else if (IDAC_off==1)               { reg.Power_Down |=  I_DACOFF; }
    
    if      (QClock_off==0)             { reg.Power_Down &= ~QCLKOFF; }
    else if (QClock_off==1)             { reg.Power_Down |=  QCLKOFF; }
    
    if      (IClock_off==0)             { reg.Power_Down &= ~ICLKOFF; }
    else if (IClock_off==1)             { reg.Power_Down |=  ICLKOFF; }
    
    if      (ExternalRef_on==0)          { reg.Power_Down &= ~EXTREF; }
    else if (ExternalRef_on==1)          { reg.Power_Down |=  EXTREF; }
    
    
    AD9117_setRegister(POWER_DOWN_REG,reg.Power_Down);
}

void AD9117_setDataControl(int8_t two_compl, int8_t I_first_data_pads, int8_t I_rising_edge, int8_t dis_simult_inp_and_output_on_DCLKIO, int8_t data_clock_input_disabled, int8_t data_clock_output_enabled, int8_t DCODBL_enabled){
    if      (two_compl==0)                { reg.Data_Control &= ~TWOS; }
    else if (two_compl==1)                { reg.Data_Control |=  TWOS; }
    
    if      (I_first_data_pads==0)             { reg.Data_Control &= ~IFIRST; }
    else if (I_first_data_pads==1)             { reg.Data_Control |=  IFIRST; }
    
    if      (I_rising_edge==0)           { reg.Data_Control &= ~IRISING; }
    else if (I_rising_edge==1)           { reg.Data_Control |=  IRISING; }
    
    if      (dis_simult_inp_and_output_on_DCLKIO==0)               { reg.Data_Control &= ~SIMULBIT; }
    else if (dis_simult_inp_and_output_on_DCLKIO==1)               { reg.Data_Control |=  SIMULBIT; }
    
    if      (data_clock_input_disabled==0)               { reg.Data_Control &= ~DCI_EN; }
    else if (data_clock_input_disabled==1)               { reg.Data_Control |=  DCI_EN; }
    
    if      (data_clock_output_enabled==0)             { reg.Data_Control &= ~DCOSGL; }
    else if (data_clock_output_enabled==1)             { reg.Data_Control |=  DCOSGL; }
    
    if      (DCODBL_enabled==0)             { reg.Data_Control &= ~DCODBL; }
    else if (DCODBL_enabled==1)             { reg.Data_Control |=  DCODBL; }
    
    AD9117_setRegister(DATA_CONTROL_REG,reg.Data_Control);
}

void AD9117_setIDacGain(uint8_t value){
    reg.I_DAC_Gain |= value & IDACGAIN;
    AD9117_setRegister(I_DAC_GAIN_REG,reg.I_DAC_Gain);
}
void AD9117_setQDacGain(uint8_t value){
    reg.Q_DAC_Gain |= value & QDACGAIN;
    AD9117_setRegister(Q_DAC_GAIN_REG,reg.Q_DAC_Gain);
}

void AD9117_setIRSET(int8_t external_resistor, int8_t internal_resistor_value){
    if(external_resistor == 0) { reg.IRSET &= ~IRSETEN;}
    if(external_resistor == 1) { reg.IRSET |=  IRSETEN;}
    if (internal_resistor_value>-1){
        reg.IRSET |= internal_resistor_value & IRSET_MASK;
    }
    AD9117_setRegister(IRSET_REG,reg.IRSET);
}

void AD9117_setQRSET(int8_t external_resistor, int8_t internal_resistor_value){
    if(external_resistor == 0) { reg.QRSET &= ~QRSETEN;}
    if(external_resistor == 1) { reg.QRSET |=  QRSETEN;}
    if (internal_resistor_value>-1){
        reg.QRSET |= internal_resistor_value & QRSET_MASK;
    }
    AD9117_setRegister(QRSET_REG,reg.QRSET);
}

void AD9117_setIRCML(int8_t external_resistor, int8_t internal_resistor_value){
    if(external_resistor == 0) { reg.IRCML &= ~IRCMLEN;}
    if(external_resistor == 1) { reg.IRCML |=  IRCMLEN;}
    if (internal_resistor_value>-1){
        reg.IRSET |= internal_resistor_value & IRCML_MASK;
    }
    AD9117_setRegister(IRCML_REG,reg.IRCML);
}

void AD9117_setQRCML(int8_t external_resistor, int8_t internal_resistor_value){
    if(external_resistor == 0) { reg.QRCML &= ~QRCMLEN;}
    if(external_resistor == 1) { reg.QRCML |=  QRCMLEN;}
    if (internal_resistor_value>-1){
        reg.QRSET |= internal_resistor_value & QRCML_MASK;
    }
    AD9117_setRegister(QRCML_REG,reg.QRCML);
}

void AD9117_setAuxDacQ(uint8_t value){
    reg.AUXDAC_Q=value;
    AD9117_setRegister(AUXDAC_Q_REG,reg.AUXDAC_Q);
}

void AD9117_setAuxCtlQ(int8_t QAuxEnabled, int8_t QAuxOutputVoltage, int8_t QAuxTopVoltage, int8_t QAuxVoltAdj){
    if(QAuxEnabled==0) {reg.AUX_CTLQ &=~QAUXEN;}
    if(QAuxEnabled==1) {reg.AUX_CTLQ |= QAUXEN;}
    
    if (QAuxOutputVoltage>-1){ reg.AUX_CTLQ |= QAuxOutputVoltage  & QAUXRNG; }
    if (QAuxTopVoltage>-1)   { reg.AUX_CTLQ |= QAuxTopVoltage     & QAUXOFS; }
    if (QAuxVoltAdj>-1)      { reg.AUX_CTLQ |= QAuxVoltAdj        & QAUXDAC_ADJ; }
    
    AD9117_setRegister(AUX_CTLQ_REG,reg.AUX_CTLQ);
}


void AD9117_setAuxDacI(uint8_t value){
    reg.AUXDAC_I=value;
    AD9117_setRegister(AUXDAC_I_REG,reg.AUXDAC_I);
}

void AD9117_setAuxCtlI(int8_t IAuxEnabled, int8_t IAuxOutputVoltage, int8_t IAuxTopVoltage, int8_t IAuxVoltAdj){
    if(IAuxEnabled==0) {reg.AUX_CTLI &=~IAUXEN;}
    if(IAuxEnabled==1) {reg.AUX_CTLI |= IAUXEN;}
    
    if (IAuxOutputVoltage>-1){ reg.AUX_CTLI |= IAuxOutputVoltage  & IAUXRNG; }
    if (IAuxTopVoltage>-1)   { reg.AUX_CTLI |= IAuxTopVoltage     & IAUXOFS; }
    if (IAuxVoltAdj>-1)      { reg.AUX_CTLI |= IAuxVoltAdj        & IAUXDAC_ADJ; }
    
    AD9117_setRegister(AUX_CTLI_REG,reg.AUX_CTLI);
}

void AD9117_setRefResistor(int8_t value){
    if (value>-1){
        reg.Reference_Resistor|=value & RREF;
    }
    AD9117_setRegister(REFERENCE_RESISTOR_REG,reg.Reference_Resistor);
}

void AD9117_setCalControl (int8_t QDacCalibSetByUser, int8_t IDacCalibSetByUser, int8_t QDacSelfCalib, int8_t IDacSelfCalib, int8_t CalibClockEnabled, int8_t ClockDivValue){
    if      (QDacCalibSetByUser==0)                { reg.Cal_Control &= ~PRELDQ; }
    else if (QDacCalibSetByUser==1)                { reg.Cal_Control |=  PRELDQ; }
    
    if      (IDacCalibSetByUser==0)             { reg.Cal_Control &= ~PRELDI; }
    else if (IDacCalibSetByUser==1)             { reg.Cal_Control |=  PRELDI; }
    
    if      (QDacSelfCalib==0)           { reg.Cal_Control &= ~CALSELQ; }
    else if (QDacSelfCalib==1)           { reg.Cal_Control |=  CALSELQ; }
    
    if      (IDacSelfCalib==0)               { reg.Cal_Control &= ~CALSELI; }
    else if (IDacSelfCalib==1)               { reg.Cal_Control |=  CALSELI; }
    
    if      (CalibClockEnabled==0)               { reg.Cal_Control &= ~CALCLK; }
    else if (CalibClockEnabled==1)               { reg.Cal_Control |=  CALCLK; }
    
    if (ClockDivValue>-1){
        reg.Cal_Control |= ClockDivValue & DIVSEL_MASK;
    }

    AD9117_setRegister (CAL_CONTROL_REG,reg.Cal_Control);
}

void AD9117_setMemoryAddress(int8_t address){
    if(address>-1){
        reg.Memory_Address |= address & MEMADDR_MASK;
    }
    AD9117_setRegister(MEMORY_ADDRESS_REG,reg.Memory_Address);
}

void AD9117_setMemoryData(int8_t data){
    if(data>-1){
        reg.Memory_Data |= data & MEMDATA_MASK;
    }
    AD9117_setRegister(MEMORY_DATA_REG,reg.Memory_Data);
}

void AD9117_setMemoryRW(int8_t clearCALSTATQ, int8_t clearCALSTATI, int8_t init_self_cal, int8_t write_to_static_memory, int8_t read_from_static_memory, int8_t reset_QDac_callib, int8_t reset_IDac_callib){
    if      (clearCALSTATQ==0)                { reg.Memory_RW &= ~CALRSTQ; }
    else if (clearCALSTATQ==1)                { reg.Memory_RW |=  CALRSTQ; }
    
    if      (clearCALSTATI==0)             { reg.Memory_RW &= ~CALRSTI; }
    else if (clearCALSTATI==1)             { reg.Memory_RW |=  CALRSTI; }
    
    if      (init_self_cal==0)              { reg.Memory_RW &= ~CALEN; }
    else if (init_self_cal==1)              { reg.Memory_RW |=  CALEN; }
    
    if      (write_to_static_memory==0)               { reg.Memory_RW &= ~SMEMWR; }
    else if (write_to_static_memory==1)               { reg.Memory_RW |=  SMEMWR; }
    
    if      (read_from_static_memory==0)               { reg.Memory_RW &= ~SMEMRD; }
    else if (read_from_static_memory==1)               { reg.Memory_RW |=  SMEMRD; }
    
    if      (reset_QDac_callib==0)               { reg.Memory_RW &= ~UNCALQ; }
    else if (reset_QDac_callib==1)               { reg.Memory_RW |=  UNCALQ; }
    
    if      (reset_IDac_callib==0)               { reg.Memory_RW &= ~UNCALI; }
    else if (reset_IDac_callib==1)               { reg.Memory_RW |=  UNCALI; }
    
    AD9117_setRegister(MEMORY_RW_REG, reg.Memory_RW);
}

void AD9117_setCLKMODE(int8_t CLKMODEQ_value, int8_t search_indic, int8_t Reacaquire_indic, int8_t CLKMODEN_value, int8_t CLKMODEI_value){
    if (CLKMODEQ_value > -1){
        reg.CLKMODE |= CLKMODEQ_value & CLKMODEQ_MASK;
    }
    
    if (CLKMODEI_value > -1){
        reg.CLKMODE |= CLKMODEI_value & CLKMODEI_MASK;
    }
    
    if      (search_indic==0)               { reg.CLKMODE &= ~Searching_MASK; }
    else if (search_indic==1)               { reg.CLKMODE |=  Searching_MASK; }
    
    if      (Reacaquire_indic==0)               { reg.CLKMODE &= ~Reacquire_MASK; }
    else if (Reacaquire_indic==1)               { reg.CLKMODE |=  Reacquire_MASK; }
    
    if      (CLKMODEN_value==0)               { reg.CLKMODE &= ~CLKMODEN_MASK; }
    else if (CLKMODEN_value==1)               { reg.CLKMODE |=  CLKMODEN_MASK; }

    AD9117_setRegister(CLKMODE_REG, reg.CLKMODE);
}