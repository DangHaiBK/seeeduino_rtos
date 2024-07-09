#include "motor.h"

/*
    * Call constructor to assign pins
*/
MotorSpeed::MotorSpeed(uint32_t motor_ip1, uint32_t motor_ip2)
{
    _ip1 = motor_ip1;
    _ip2 = motor_ip2;
}

/*
    * Initialize motor
*/
void MotorSpeed::begin()
{
    pinMode(_ip1, OUTPUT);
    pinMode(_ip2, OUTPUT);
}

/* 
    * Stop motor
*/
void MotorSpeed::SetStop()
{
    digitalWrite(_ip1, LOW);
    digitalWrite(_ip2, LOW);
}

/* 
    * Set full speed when forward
*/
void MotorSpeed::SetFullForward()
{
    digitalWrite(_ip1, LOW);
    analogWrite(_ip2, 230);
}

/* 
    * Set full speed when reversed
*/
void MotorSpeed::SetFullReversed()
{
    analogWrite(_ip1, 127);
    digitalWrite(_ip2, LOW);
}

/* 
    * Set PwM value for motor when forward
    * Params:
    * - val: PWM value
*/
void MotorSpeed::SetPwmForward(uint16_t val)
{
    digitalWrite(_ip1, LOW);
    analogWrite(_ip2, val);
}

/* 
    * Set PwM value for motor when reversed
    * Params:
    * - val: PWM value
*/
void MotorSpeed::SetPwmReversed(uint16_t val)
{
    analogWrite(_ip1, val);
    digitalWrite(_ip2, LOW);
}