#ifndef _CAR_INIT_H_
#define _CAR_INIT_H_

#include "common.h"
#include "car_global.h"
	
void init_all();
void init_gpio();
void pwm_init();
void adc_init();
void uart_interr_init();
void pit_init();
void init_paranum();
void init_readpara();
void init_all_pulse_counter();
void init_i2c();
void init_sdhc();
void init_setpara();
void dma_count_init();

#endif