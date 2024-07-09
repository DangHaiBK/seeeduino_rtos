#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class MotorSpeed
{
public:
    MotorSpeed(uint32_t motor_ip1, uint32_t motor_ip2);
    void begin();
    void SetFullForward();
    void SetFullReversed();
    void SetStop();
    void SetPwmForward(uint16_t val);
    void SetPwmReversed(uint16_t val);

private:
    /* data */
    uint32_t _ip1;
    uint32_t _ip2;
};

#endif /* MOTOR_H */