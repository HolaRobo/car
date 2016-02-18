#ifndef _CAR_ISR_H_
#define _CAR_ISR_H_
#include "common.h"
#include "car_global.h"

void portc_isr();
void uart_isr();
void pit0_isr();
void change_para(char event); 
void IMU_Update(float Gyro,float Acc);
void Upright_Control_calculate();
void MotorOutput();
void SpeedControl();
void SpeedControlOutput(); 
void DirectionControl();
void DirectionControlOutput();

#endif
