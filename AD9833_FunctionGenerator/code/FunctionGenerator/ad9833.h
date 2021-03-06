#ifndef _AD9833_H_
#define _AD9833_H_

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define AD_FSYNC_PORT PORTB
#define AD_FSYNC_DDR  DDRB
#define AD_FSYNC_BIT  PB2

#define AD_FSYNC_HI() AD_FSYNC_PORT |=  (1<<AD_FSYNC_BIT)
#define AD_FSYNC_LO() AD_FSYNC_PORT &= ~(1<<AD_FSYNC_BIT)

#define AD_OFF      0
#define AD_TRIANGLE 1
#define AD_SQUARE   2
#define AD_SINE     3

#define AD_B28     13
#define AD_HLB     12
#define AD_FSELECT 11
#define AD_PSELECT 10
#define AD_RESET   8
#define AD_SLEEP1  7
#define AD_SLEEP12 6
#define AD_OPBITEN 5
#define AD_DIV2    3
#define AD_MODE    1

#define AD_FREQ0  (1<<14)
#define AD_FREQ1  (1<<15)
#define AD_PHASE0 (3<<14)
#define AD_PHASE1 ((3<<14)|(1<<13))

#define AD_F_MCLK 20000000UL
#define AD_2POW28 268435456UL


#define AD_FREQ_CALC(freq) (uint32_t)(((double)AD_2POW28/(double)AD_F_MCLK*freq)*4)

#define AD_MOD_FREQ_CALC(freq) (F_CPU/(64*(uint32_t)freq))

#define AD_PHASE_CALC(phase_deg) (uint16_t)((512*phase_deg)/45)

#define TIMER_START() ICCR1B |=   (1<<CS11)|(1<<CS10)
#define TIMER_STOP()  ICCR1B &= ~((1<<CS11)|(1<<CS10))

typedef struct {
     float    freq[2];
     float    phase[2];
     float    mod_freq;
     uint8_t  freq_out;
     uint8_t  phase_out;
     uint8_t  mode;
     uint16_t command_reg;
} ad9833_settings_t;

//some functions could be done more efficiently, maybe as inline functions...

void ad9833_init(void);

void ad9833_set_mode(uint8_t mode);

void    ad9833_set_frequency(uint8_t reg, double freq);
//double  ad9833_get_frequency(uint8_t reg);

void    ad9833_set_phase(uint8_t reg, double phase);
//double  ad9833_get_phase(uint8_t reg);

void    ad9833_set_freq_out(uint8_t freq_out);
uint8_t ad9833_get_freq_out(void);

void    ad9833_set_phase_out(uint8_t phase_out);
uint8_t ad9833_get_phase_out(void);

void    ad9833_set_mod_freq(uint16_t freq);
void    ad9833_set_mod_bytes(uint8_t num, uint8_t *bytes, uint8_t repeat);

#endif