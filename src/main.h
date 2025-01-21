#ifndef __MAIN_H__
#define __MAIN_H__

#include "rx.h"
#include "led.h"
#include "motor.h"

#define RX_TEST_ONLY                    1
#define SERIAL_DEBUG                    1

#define RX_COMPENSATE_ERROR_ENABLE      1
#define RX_COMPENSATE_ERROR_DISABLE     0      

#define RX_CHANNEL_1                    RX_INPUT_MAP_CHANNEL_1_TO_NUMBER
#define RX_CHANNEL_2                    RX_INPUT_MAP_CHANNEL_2_TO_NUMBER
#define RX_CHANNEL_3                    RX_INPUT_MAP_CHANNEL_3_TO_NUMBER

#define MINIMUM_MOTOR_SPEED             0
#define MAXIMUM_MOTOR_SPEED_FORWARD     230
#define MAXIMUM_MOTOR_SPEED_REVERSED    127

#define TIME_ACTIVE_STEERING            2000      // In milli-second

#define DELAY_RX_TASK                   25        // In milli-second
#define DELAY_MAIN_TASK                 100

#define LIGHT_PERIOD                    ((LED_TOGGLE_PERIOD / DELAY_RX_TASK))
#define LIGHT_BRAKE_PERIOD              ((LED_TOGGLE_PERIOD / DELAY_RX_TASK))
#define LIGHT_PERIOD_FAILSAFE           ((LED_TOGGLE_PERIOD_FAILSAFE / DELAY_RX_TASK))
#define TIME_THRES_STEERING             ((TIME_ACTIVE_STEERING / DELAY_RX_TASK))

#endif /* __MAIN_H__ */