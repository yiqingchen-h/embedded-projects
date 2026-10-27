#ifndef __SYS_H
#define __SYS_H		
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "Re.h"
#include "Switch.h"
#include "Stepper_Motors.h"
#include "LED.h"
#include "Uart.h"
#include "IIC.h"
#include "Trace.h"
#include "Servos.h"
#include "IMU.h"
#include "bus_Servos_Gimbal.h"
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t


void delay_ms(unsigned int ms);
void delay_us(unsigned int us);
void Electromagnets_ON(void);
void Electromagnets_OFF(void);
#endif