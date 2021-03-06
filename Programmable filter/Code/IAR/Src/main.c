#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "AD9117.h"

static void SystemClock_Config(void);
static void Error_Handler(void);
char uart_sendChar(char ch);
uint16_t uart_readChar();
void parseReceivedData(char *data);

UART_HandleTypeDef UartHandle;
GPIO_InitTypeDef GPIO_InitStruct_ADC;
TIM_HandleTypeDef        TimHandle;
GPIO_InitTypeDef  GPIO_InitStruct_mDAC;
GPIO_InitTypeDef GPIO_InitStruct_Int;

#define MDAC_OUT_SYNC       GPIO_PIN_14 //PORTB
#define MDAC_OUT_SDIN       GPIO_PIN_15 //PORTB
#define MDAC_OUT_SCLK       GPIO_PIN_13 //PORTB

#define MDAC_IN_SYNC       GPIO_PIN_2  //PORTD
#define MDAC_IN_SYNC_PORT  GPIOD       //PORTD
#define MDAC_IN_SDIN       GPIO_PIN_15  //PORTB
#define MDAC_IN_SCLK       GPIO_PIN_13  //PORTB

#define DATA_BUFF_SIZE 50
#define FILTER_MAX_ORDER 50

#define UART_RX_TERMINATOR '\n'

//------------------------------------------//
//------------ADC functions-----------------//
//------------------------------------------//
void AD9245_init(){
  GPIO_InitStruct_ADC.Pin = 0xFFFF;
  GPIO_InitStruct_ADC.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct_ADC.Pull = GPIO_NOPULL;
  GPIO_InitStruct_ADC.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_ADC); 
}
inline uint16_t AD9245_getValue(){      
  //this function takes 2 clock cycles
  uint16_t ret = (GPIOC->IDR & 0x7FFF);
  return ret;
}

//----------------------------------------//
//-----------Test functions--------------//
//---------------------------------------//
void Test_init(){
  GPIO_InitStruct_ADC.Pin = 0x8000;
  GPIO_InitStruct_ADC.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_ADC.Pull = GPIO_NOPULL;
  GPIO_InitStruct_ADC.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_ADC); 
}
uint16_t sinTable[] = {
  0x2000,0x2032,0x2065,0x2097,0x20c9,0x20fc,0x212e,0x2160,
0x2192,0x21c5,0x21f7,0x2229,0x225b,0x228d,0x22c0,0x22f2,
0x2324,0x2356,0x2388,0x23ba,0x23ec,0x241e,0x2450,0x2481,
0x24b3,0x24e5,0x2517,0x2548,0x257a,0x25ab,0x25dd,0x260e,
0x2640,0x2671,0x26a2,0x26d3,0x2705,0x2736,0x2767,0x2798,
0x27c8,0x27f9,0x282a,0x285a,0x288b,0x28bb,0x28ec,0x291c,
0x294c,0x297c,0x29ac,0x29dc,0x2a0c,0x2a3c,0x2a6b,0x2a9b,
0x2aca,0x2afa,0x2b29,0x2b58,0x2b87,0x2bb6,0x2be5,0x2c13,
0x2c42,0x2c70,0x2c9f,0x2ccd,0x2cfb,0x2d29,0x2d57,0x2d84,
0x2db2,0x2ddf,0x2e0c,0x2e3a,0x2e67,0x2e93,0x2ec0,0x2eed,
0x2f19,0x2f45,0x2f72,0x2f9e,0x2fc9,0x2ff5,0x3021,0x304c,
0x3077,0x30a2,0x30cd,0x30f8,0x3123,0x314d,0x3177,0x31a1,
0x31cb,0x31f5,0x321e,0x3248,0x3271,0x329a,0x32c3,0x32ec,
0x3314,0x333c,0x3364,0x338c,0x33b4,0x33dc,0x3403,0x342a,
0x3451,0x3478,0x349f,0x34c5,0x34eb,0x3511,0x3537,0x355c,
0x3582,0x35a7,0x35cc,0x35f1,0x3615,0x3639,0x365e,0x3681,
0x36a5,0x36c9,0x36ec,0x370f,0x3732,0x3754,0x3776,0x3798,
0x37ba,0x37dc,0x37fd,0x381f,0x3840,0x3860,0x3881,0x38a1,
0x38c1,0x38e1,0x3900,0x3920,0x393f,0x395d,0x397c,0x399a,
0x39b8,0x39d6,0x39f4,0x3a11,0x3a2e,0x3a4b,0x3a67,0x3a84,
0x3aa0,0x3abc,0x3ad7,0x3af2,0x3b0d,0x3b28,0x3b43,0x3b5d,
0x3b77,0x3b90,0x3baa,0x3bc3,0x3bdc,0x3bf5,0x3c0d,0x3c25,
0x3c3d,0x3c54,0x3c6c,0x3c83,0x3c99,0x3cb0,0x3cc6,0x3cdc,
0x3cf1,0x3d07,0x3d1c,0x3d30,0x3d45,0x3d59,0x3d6d,0x3d81,
0x3d94,0x3da7,0x3dba,0x3dcc,0x3ddf,0x3df0,0x3e02,0x3e13,
0x3e24,0x3e35,0x3e46,0x3e56,0x3e66,0x3e75,0x3e85,0x3e94,
0x3ea2,0x3eb1,0x3ebf,0x3ecd,0x3eda,0x3ee7,0x3ef4,0x3f01,
0x3f0d,0x3f19,0x3f25,0x3f30,0x3f3b,0x3f46,0x3f51,0x3f5b,
0x3f65,0x3f6e,0x3f78,0x3f81,0x3f89,0x3f92,0x3f9a,0x3fa2,
0x3fa9,0x3fb0,0x3fb7,0x3fbe,0x3fc4,0x3fca,0x3fcf,0x3fd5,
0x3fda,0x3fde,0x3fe3,0x3fe7,0x3feb,0x3fee,0x3ff1,0x3ff4,
0x3ff7,0x3ff9,0x3ffb,0x3ffd,0x3ffe,0x3fff,0x3fff,0x3fff,
0x3fff,0x3fff,0x3fff,0x3ffe,0x3ffd,0x3ffc,0x3ffa,0x3ff8,
0x3ff5,0x3ff3,0x3ff0,0x3fec,0x3fe9,0x3fe5,0x3fe1,0x3fdc,
0x3fd7,0x3fd2,0x3fcd,0x3fc7,0x3fc1,0x3fba,0x3fb4,0x3fad,
0x3fa5,0x3f9e,0x3f96,0x3f8e,0x3f85,0x3f7c,0x3f73,0x3f6a,
0x3f60,0x3f56,0x3f4b,0x3f41,0x3f36,0x3f2b,0x3f1f,0x3f13,
0x3f07,0x3efa,0x3eee,0x3ee1,0x3ed3,0x3ec6,0x3eb8,0x3ea9,
0x3e9b,0x3e8c,0x3e7d,0x3e6d,0x3e5e,0x3e4e,0x3e3d,0x3e2d,
0x3e1c,0x3e0b,0x3df9,0x3de8,0x3dd5,0x3dc3,0x3db1,0x3d9e,
0x3d8a,0x3d77,0x3d63,0x3d4f,0x3d3b,0x3d26,0x3d11,0x3cfc,
0x3ce7,0x3cd1,0x3cbb,0x3ca5,0x3c8e,0x3c77,0x3c60,0x3c49,
0x3c31,0x3c19,0x3c01,0x3be8,0x3bd0,0x3bb6,0x3b9d,0x3b84,
0x3b6a,0x3b50,0x3b35,0x3b1b,0x3b00,0x3ae5,0x3ac9,0x3aae,
0x3a92,0x3a76,0x3a59,0x3a3d,0x3a20,0x3a02,0x39e5,0x39c7,
0x39a9,0x398b,0x396d,0x394e,0x392f,0x3910,0x38f1,0x38d1,
0x38b1,0x3891,0x3871,0x3850,0x382f,0x380e,0x37ed,0x37cb,
0x37a9,0x3787,0x3765,0x3743,0x3720,0x36fd,0x36da,0x36b7,
0x3693,0x366f,0x364c,0x3627,0x3603,0x35de,0x35b9,0x3594,
0x356f,0x354a,0x3524,0x34fe,0x34d8,0x34b2,0x348b,0x3465,
0x343e,0x3417,0x33ef,0x33c8,0x33a0,0x3378,0x3350,0x3328,
0x3300,0x32d7,0x32af,0x3286,0x325c,0x3233,0x320a,0x31e0,
0x31b6,0x318c,0x3162,0x3138,0x310d,0x30e3,0x30b8,0x308d,
0x3062,0x3036,0x300b,0x2fdf,0x2fb4,0x2f88,0x2f5c,0x2f2f,
0x2f03,0x2ed6,0x2eaa,0x2e7d,0x2e50,0x2e23,0x2df6,0x2dc8,
0x2d9b,0x2d6d,0x2d40,0x2d12,0x2ce4,0x2cb6,0x2c87,0x2c59,
0x2c2b,0x2bfc,0x2bcd,0x2b9e,0x2b70,0x2b40,0x2b11,0x2ae2,
0x2ab3,0x2a83,0x2a54,0x2a24,0x29f4,0x29c4,0x2994,0x2964,
0x2934,0x2904,0x28d4,0x28a3,0x2873,0x2842,0x2812,0x27e1,
0x27b0,0x277f,0x274e,0x271d,0x26ec,0x26bb,0x268a,0x2658,
0x2627,0x25f6,0x25c4,0x2593,0x2561,0x252f,0x24fe,0x24cc,
0x249a,0x2468,0x2437,0x2405,0x23d3,0x23a1,0x236f,0x233d,
0x230b,0x22d9,0x22a6,0x2274,0x2242,0x2210,0x21de,0x21ab,
0x2179,0x2147,0x2115,0x20e2,0x20b0,0x207e,0x204b,0x2019,
0x1fe7,0x1fb5,0x1f82,0x1f50,0x1f1e,0x1eeb,0x1eb9,0x1e87,
0x1e55,0x1e22,0x1df0,0x1dbe,0x1d8c,0x1d5a,0x1d27,0x1cf5,
0x1cc3,0x1c91,0x1c5f,0x1c2d,0x1bfb,0x1bc9,0x1b98,0x1b66,
0x1b34,0x1b02,0x1ad1,0x1a9f,0x1a6d,0x1a3c,0x1a0a,0x19d9,
0x19a8,0x1976,0x1945,0x1914,0x18e3,0x18b2,0x1881,0x1850,
0x181f,0x17ee,0x17be,0x178d,0x175d,0x172c,0x16fc,0x16cc,
0x169c,0x166c,0x163c,0x160c,0x15dc,0x15ac,0x157d,0x154d,
0x151e,0x14ef,0x14c0,0x1490,0x1462,0x1433,0x1404,0x13d5,
0x13a7,0x1379,0x134a,0x131c,0x12ee,0x12c0,0x1293,0x1265,
0x1238,0x120a,0x11dd,0x11b0,0x1183,0x1156,0x112a,0x10fd,
0x10d1,0x10a4,0x1078,0x104c,0x1021,0xff5,0xfca,0xf9e,
0xf73,0xf48,0xf1d,0xef3,0xec8,0xe9e,0xe74,0xe4a,
0xe20,0xdf6,0xdcd,0xda4,0xd7a,0xd51,0xd29,0xd00,
0xcd8,0xcb0,0xc88,0xc60,0xc38,0xc11,0xbe9,0xbc2,
0xb9b,0xb75,0xb4e,0xb28,0xb02,0xadc,0xab6,0xa91,
0xa6c,0xa47,0xa22,0x9fd,0x9d9,0x9b4,0x991,0x96d,
0x949,0x926,0x903,0x8e0,0x8bd,0x89b,0x879,0x857,
0x835,0x813,0x7f2,0x7d1,0x7b0,0x78f,0x76f,0x74f,
0x72f,0x70f,0x6f0,0x6d1,0x6b2,0x693,0x675,0x657,
0x639,0x61b,0x5fe,0x5e0,0x5c3,0x5a7,0x58a,0x56e,
0x552,0x537,0x51b,0x500,0x4e5,0x4cb,0x4b0,0x496,
0x47c,0x463,0x44a,0x430,0x418,0x3ff,0x3e7,0x3cf,
0x3b7,0x3a0,0x389,0x372,0x35b,0x345,0x32f,0x319,
0x304,0x2ef,0x2da,0x2c5,0x2b1,0x29d,0x289,0x276,
0x262,0x24f,0x23d,0x22b,0x218,0x207,0x1f5,0x1e4,
0x1d3,0x1c3,0x1b2,0x1a2,0x193,0x183,0x174,0x165,
0x157,0x148,0x13a,0x12d,0x11f,0x112,0x106,0xf9,
0xed,0xe1,0xd5,0xca,0xbf,0xb5,0xaa,0xa0,
0x96,0x8d,0x84,0x7b,0x72,0x6a,0x62,0x5b,
0x53,0x4c,0x46,0x3f,0x39,0x33,0x2e,0x29,
0x24,0x1f,0x1b,0x17,0x14,0x10,0xd,0xb,
0x8,0x6,0x4,0x3,0x2,0x1,0x0,0x0,
0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,
0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,
0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,
0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,
0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,
0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,
0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,
0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,0x2000,
0x0,0x0,0x1,0x2,0x3,0x5,0x7,0x9,
0xc,0xf,0x12,0x15,0x19,0x1d,0x22,0x26,
0x2b,0x31,0x36,0x3c,0x42,0x49,0x50,0x57,
0x5e,0x66,0x6e,0x77,0x7f,0x88,0x92,0x9b,
0xa5,0xaf,0xba,0xc5,0xd0,0xdb,0xe7,0xf3,
0xff,0x10c,0x119,0x126,0x133,0x141,0x14f,0x15e,
0x16c,0x17b,0x18b,0x19a,0x1aa,0x1ba,0x1cb,0x1dc,
0x1ed,0x1fe,0x210,0x221,0x234,0x246,0x259,0x26c,
0x27f,0x293,0x2a7,0x2bb,0x2d0,0x2e4,0x2f9,0x30f,
0x324,0x33a,0x350,0x367,0x37d,0x394,0x3ac,0x3c3,
0x3db,0x3f3,0x40b,0x424,0x43d,0x456,0x470,0x489,
0x4a3,0x4bd,0x4d8,0x4f3,0x50e,0x529,0x544,0x560,
0x57c,0x599,0x5b5,0x5d2,0x5ef,0x60c,0x62a,0x648,
0x666,0x684,0x6a3,0x6c1,0x6e0,0x700,0x71f,0x73f,
0x75f,0x77f,0x7a0,0x7c0,0x7e1,0x803,0x824,0x846,
0x868,0x88a,0x8ac,0x8ce,0x8f1,0x914,0x937,0x95b,
0x97f,0x9a2,0x9c7,0x9eb,0xa0f,0xa34,0xa59,0xa7e,
0xaa4,0xac9,0xaef,0xb15,0xb3b,0xb61,0xb88,0xbaf,
0xbd6,0xbfd,0xc24,0xc4c,0xc74,0xc9c,0xcc4,0xcec,
0xd14,0xd3d,0xd66,0xd8f,0xdb8,0xde2,0xe0b,0xe35,
0xe5f,0xe89,0xeb3,0xedd,0xf08,0xf33,0xf5e,0xf89,
0xfb4,0xfdf,0x100b,0x1037,0x1062,0x108e,0x10bb,0x10e7,
0x1113,0x1140,0x116d,0x1199,0x11c6,0x11f4,0x1221,0x124e,
0x127c,0x12a9,0x12d7,0x1305,0x1333,0x1361,0x1390,0x13be,
0x13ed,0x141b,0x144a,0x1479,0x14a8,0x14d7,0x1506,0x1536,
0x1565,0x1595,0x15c4,0x15f4,0x1624,0x1654,0x1684,0x16b4,
0x16e4,0x1714,0x1745,0x1775,0x17a6,0x17d6,0x1807,0x1838,
0x1868,0x1899,0x18ca,0x18fb,0x192d,0x195e,0x198f,0x19c0,
0x19f2,0x1a23,0x1a55,0x1a86,0x1ab8,0x1ae9,0x1b1b,0x1b4d,
0x1b7f,0x1bb0,0x1be2,0x1c14,0x1c46,0x1c78,0x1caa,0x1cdc,
0x1d0e,0x1d40,0x1d73,0x1da5,0x1dd7,0x1e09,0x1e3b,0x1e6e,
0x1ea0,0x1ed2,0x1f04,0x1f37,0x1f69,0x1f9b,0x1fce,0x2000};

//-------------------------------------------//
//-----------------Timer---------------------//
//-------------------------------------------//
void init_timer(){
  uint32_t TickPriority=1;
  
  RCC_ClkInitTypeDef sClokConfig;
  uint32_t uwTimclock, uwAPB1Prescaler = 0;
  uint32_t pFLatency;
  
    /*Configure the TIM5 IRQ priority */
  HAL_NVIC_SetPriority(TIM5_IRQn, TickPriority ,0); 
  
  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&sClokConfig, &pFLatency);
  
  /* Get APB1 prescaler */
  uwAPB1Prescaler = sClokConfig.APB1CLKDivider;
  
  /* Compute TIM5 clock */
  if (uwAPB1Prescaler == 0) 
  {
    uwTimclock = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    uwTimclock = 2*HAL_RCC_GetPCLK1Freq();
  }

  /* Initialize TIM5 */
  TimHandle.Instance = TIM5;
    
  /* Initialize TIMx peripheral as follow:
       + Period = [(TIM5CLK/1000) - 1]. to have a (1/1000) s time base.
       + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period = 0x110;
  TimHandle.Init.Prescaler = 0;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Start the TIM time Base generation in interrupt mode */
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  
}

//-------------------------------------------//
//-----------------mDAC---------------------//
//------------------------------------------//
void init_mDAC_output(){
  //pins initialization
  GPIO_InitStruct_ADC.Pin = MDAC_OUT_SDIN | MDAC_OUT_SYNC | MDAC_OUT_SCLK;
  GPIO_InitStruct_ADC.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_ADC.Pull = GPIO_NOPULL;
  GPIO_InitStruct_ADC.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_ADC); 
  
  wait_ms(10);
  //set all pins by default to High
  HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SYNC,GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SCLK,GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SDIN,GPIO_PIN_SET); 
  for (int i=0;i<100;i++) wait_ms(10);
}
void mDAC_output(uint16_t data){ //max 16383
  HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SDIN,GPIO_PIN_RESET); 
  
  HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SYNC,GPIO_PIN_RESET); 
  
  for (int i=15;i>-1;i--){
    if (data & (1<<i)){
      HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SDIN,GPIO_PIN_SET); 
    } else {
      HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SDIN,GPIO_PIN_RESET); 
    }
    HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SCLK,GPIO_PIN_RESET); 
    
    HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SCLK,GPIO_PIN_SET); 
  }
  
  HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SYNC,GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOB,MDAC_OUT_SDIN,GPIO_PIN_SET); 
}

void init_mDAC_input(){
  
  //SDIN and SCLK are initialized in mDAC_output();
  
  GPIO_InitTypeDef  GPIO_InitStruct_DAC;
  
  //pins initialization
  GPIO_InitStruct_DAC.Pin = MDAC_IN_SYNC;
  GPIO_InitStruct_DAC.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_DAC.Pull = GPIO_NOPULL;
  GPIO_InitStruct_DAC.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(MDAC_IN_SYNC_PORT, &GPIO_InitStruct_DAC); 
  
  wait_ms(10);
  //set all pins by default to High
  HAL_GPIO_WritePin(MDAC_IN_SYNC_PORT,MDAC_IN_SYNC,GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOB,MDAC_IN_SCLK,GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOB,MDAC_IN_SDIN,GPIO_PIN_SET); 
  for (int i=0;i<100;i++) wait_ms(10);
}
void mDAC_input(uint16_t data){ //max 16383
  HAL_GPIO_WritePin(GPIOB,MDAC_IN_SDIN,GPIO_PIN_RESET); 
  
  HAL_GPIO_WritePin(MDAC_IN_SYNC_PORT,MDAC_IN_SYNC,GPIO_PIN_RESET); 
  
  for (int i=15;i>-1;i--){
    if (data & (1<<i)){
      HAL_GPIO_WritePin(GPIOB,MDAC_IN_SDIN,GPIO_PIN_SET); 
    } else {
      HAL_GPIO_WritePin(GPIOB,MDAC_IN_SDIN,GPIO_PIN_RESET); 
    }
    HAL_GPIO_WritePin(GPIOB,MDAC_IN_SCLK,GPIO_PIN_RESET); 
    
    HAL_GPIO_WritePin(GPIOB,MDAC_IN_SCLK,GPIO_PIN_SET); 
  }
  
  HAL_GPIO_WritePin(MDAC_IN_SYNC_PORT,MDAC_IN_SYNC,GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOB,MDAC_IN_SDIN,GPIO_PIN_SET); 
}
//---------------------------------------//
//----------------UART------------------//
//-------------------------------------//
void initUART(){
  
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None parity
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;
  
  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();
  
  /* Enable USARTx clock */
  USARTx_CLK_ENABLE(); 
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;
  
  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
    
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;
    
  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
 
}
void getUserCommand(){
  initUART();
  
  char dataBuffer[30];
  for (int i=0;i<30;i++) dataBuffer[i]=0;
  int i_dataBuffer=0;
  
  while(GPIOB->IDR & GPIO_PIN_0){

    //read char and write it back to the user
    uint16_t ch = uart_readChar();
    if(ch<256){  
      uart_sendChar((uint8_t)ch);  
      dataBuffer[i_dataBuffer]=ch;
      i_dataBuffer++;
      if(i_dataBuffer>30) i_dataBuffer=29;
      
      if(ch==UART_RX_TERMINATOR){ parseReceivedData(dataBuffer); i_dataBuffer=0;}
    }
    
  }
  AD9117_init();
}
void initIntButt(){
  GPIO_InitStruct_Int.Pin = GPIO_PIN_0;
  GPIO_InitStruct_Int.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct_Int.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct_Int.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_Int); 
}
/*Send one char to the user using UART*/
char uart_sendChar(char ch){
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0x100); 
  return ch;
}
uint16_t uart_readChar(){
   uint16_t ch=0;
   if (HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 0x1000)!=HAL_OK){
    ch=257;
   }
   return ch;
}

#define NUM_SIZE 50
float num[NUM_SIZE];      //TODO change to float
int num_order=0;

#define DEN_SIZE 50
float den[DEN_SIZE];      //TODO change to float
int den_order=0;

void parseReceivedData(char *data){
  
  char numb[10];
  char numb2[10];
  for (int i=0;i<10;i++) numb[i]=0;
  
  int i_numb=0;
  int bCharPos=0;
  for (int i=0;i<30;i++){
    bCharPos=i;
    if(data[i]==UART_RX_TERMINATOR || data[i]==':') break;
    if(data[i]>='0' && data[i]<='9'){
      numb[i_numb]=data[i];
      i_numb++;
      if(i_numb==9) break;
    }
  }
  numb[i_numb]=0;
  
  int i_numb2=0;
  for (int i=bCharPos;i<30;i++){
    if(data[i]==UART_RX_TERMINATOR) break;
    if(data[i]>='0' && data[i]<='9' || data[i]=='.' || data[i]==',' || data[i]=='-'){
      if(data[i]==',') data[i]='.'; //replace comma (,) with a dot (.)
      numb2[i_numb2]=data[i];
      i_numb2++;
      if(i_numb2==9) break;
    }
  }
  numb2[i_numb2]=0;
  
  int intValue = atoi(numb);
  int intValue2 = atoi(numb2);
  float floatValue2 = atof(numb2);
  
  //    numerator
  //    ---------
  //   denumerator
  
  switch (data[0]){
  case 'G':     //output mDAC gain adjustment
    mDAC_output((uint16_t)intValue2%16384);
    break;
  case 'g':     //input mDAC gain adjustment
    mDAC_input((uint16_t)intValue2%16384);
    break;
  case 'n':  //numerator
    if(intValue<NUM_SIZE){
      num[intValue]=floatValue2;
    }
    break;
  case 'd':  //denumerator
    if(intValue<DEN_SIZE){
      den[intValue]=floatValue2;
    }
    break;
  case 'N':  //numerator order
    if(intValue<NUM_SIZE){
      num_order=intValue2;
      uart_sendChar(num_order);
    }
    break;
  case 'D':  //denumerator order
    if(intValue<DEN_SIZE){
      den_order=intValue2;
      uart_sendChar(den_order);
    }
    break;
  case 'S':    //print out current filter coefficients
    for (int i=0;i<num_order;i++){
      char buff[30];
      for (int ii=0;ii<30;ii++) buff[ii]=0;
      sprintf(buff,"%d%d ",(int)(num[i]*1000),(int)(abs((long int)(num[i]*10000000)%10000)));
      for (int ii=0;ii<30;ii++){
        if(buff[ii]==0) break;
        uart_sendChar(buff[ii]);
      }
    }
    uart_sendChar('\n');
    uart_sendChar('-');uart_sendChar('-');uart_sendChar('-');uart_sendChar('-');uart_sendChar('-');uart_sendChar('-');
    uart_sendChar('\n');
        for (int i=0;i<den_order;i++){
      char buff[30];
      for (int ii=0;ii<30;ii++) buff[ii]=0;
      sprintf(buff,"%d%d ",(int)(den[i]*1000),(int)(abs((long int)(den[i]*10000000)%10000)));
      for (int ii=0;ii<30;ii++){
        if(buff[ii]==0) break;
        uart_sendChar(buff[ii]);
      }
    }
    break;
  }
}

//-------------------------------------//
//--------------MAIN-------------------//
//-------------------------------------//
int main(void){
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4 
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 100 Mhz */
  SystemClock_Config();
  
  /*##-1- Enable GPIOA Clock (to be able to program the configuration registers) */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  
  /*Initialized ports and devices*/
  AD9117_init();
  AD9245_init();
  init_mDAC_output();
  init_mDAC_input();
  initIntButt();
  
  /*set the initial mDacOutputValue*/
  uint16_t mDacOutputValue=11000;
  mDAC_output(mDacOutputValue);
  
  /*set the initial mDacInputValue*/
  uint16_t mDacInputValue=9600;
  mDAC_input(mDacInputValue);
  
  /*Init timer*/
  __TIM5_CLK_ENABLE();
  init_timer();
  
  /*data buffer initialization*/
  float data_input[DATA_BUFF_SIZE];
  for (int i=0;i<DATA_BUFF_SIZE;++i){ data_input[i]=0; }    //initialization with zeros
  uint16_t i_data_input=0;
  
  /*filter array initialization*/
  /*float filter_coeff[FILTER_MAX_ORDER];
  for (int i=0;i<FILTER_MAX_ORDER;++i){ filter_coeff[i]=0; } filter_coeff[0]=1; //filter initialy is set to pass through the data
  uint16_t filter_selected_order=6; 
  uint32_t counter=0;*/
  
  for (int i=0;i<DEN_SIZE;++i){ den[i]=0; } den[0]=1;
  den_order=6;
  for (int i=0;i<NUM_SIZE;++i){ num[i]=0; } num[0]=1;
  num_order=6;
  
  while (1)
  {
    
    /*start timer to count loop execution time*/
    __HAL_TIM_SetCounter(&TimHandle,0); 

    /*read value from the ADC*/
    uint16_t value = AD9245_getValue();
    
    /*interrupt button to stops the filter execution and read user commands
      it is not implemented as an interrupt because it will require filter_coeff to be define as volatile which requires more cycles to read.*/
    while(GPIOB->IDR & GPIO_PIN_0) {getUserCommand();};
    
    /*Test values for the DAC*/
    //value = sinTable[counter];
    //counter++;
    //if(counter>1024/*16383*/) { counter=0; /*mDacValue++; if(mDacValue>16384) mDacValue=0; mDAC(mDacValue);*/}
    
    /*Calculate the filter output*/
    uint16_t i_d=i_data_input;
    data_input[i_data_input++]=(float)(value&0x7FFF);
    if(i_data_input>=DATA_BUFF_SIZE) i_data_input=0;
    float fOutput=0;
    uint16_t i_f=0;
    for (;i_f<den_order;++i_f,--i_d){
      fOutput+=den[i_f]*data_input[i_d];
      if(i_d==0) i_d=DATA_BUFF_SIZE;
    }
    uint16_t uiOutput=(uint16_t)fOutput;
    
    /*Output teh result to the DAC*/
    #define AD9117_DCLKIO_PIN GPIO_PIN_9
    GPIOB->BSRRL = AD9117_DCLKIO_PIN;
    GPIOA->BSRRL = uiOutput;
    GPIOA->BSRRH = ~(uiOutput);
    GPIOB->BSRRH = AD9117_DCLKIO_PIN;
    
    while (__HAL_TIM_GetCounter(&TimHandle)<100) {}     //85
  }
}

/*Setup the system clock*/
static void SystemClock_Config(void){
  
  /**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
  
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;//12;  16
  RCC_OscInitStruct.PLL.PLLN = 400;//511;   400
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;//5;   7
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Error_Handler(void){
  /**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
  
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line){ 
  /**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
  
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
