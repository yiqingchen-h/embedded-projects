#ifndef ALLHEADER_H_
#define ALLHEADER_H_

#include <stdio.h>

#include "software/takes.h"
#include "software/timer.h"
#include "software/UART.h"
#include "software/display_speed_control.h"
#include "software/mission_task.h"
#include "software/voice_dataset_capture.h"
#include "software/voice_ai.h"
#include "hardware/mechanical_arm/Mechanical_arm_feedback.h"
#include "software/SDRAM/SDRAM.h"
#include "software/ui/lcd_show.h"

#include "hardware/Chassis/Chassis.h"
#include "hardware/Chassis/Chassis_config.h"
#include "hardware/Chassis/Chassis_pid.h"
#include "hardware/Chassis/chassis_avoidance.h"
#include "hardware/Chassis/chassis_param.h"
#include "hardware/Chassis/encoder.h"
#include "hardware/Chassis/kinematics.h"
#include "hardware/Chassis/motor.h"
#include "hardware/Chassis/uart_motor.h"
#include "hardware/LCD/lcd_mipi_st7701p.h"
#include "hardware/LCD/lcd_st7789.h"
#include "hardware/Flash/flash.h"
#include "hardware/INA226/ina226.h"
#include "hardware/MAX98357A/max98357a.h"
#include "hardware/Microphone/microphone.h"
#include "hardware/N10/n10_radar.h"
#include "hardware/key/key.h"
#include "hardware/mechanical_arm/Bus_servo.h"
#include "hardware/mechanical_arm/Calibration.h"
#include "hardware/mechanical_arm/Mechanical_hand_control.h"
#include "hardware/vision/Vision_Uart8_Comm.h"

#endif /* ALLHEADER_H_ */
