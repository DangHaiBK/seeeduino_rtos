#ifndef LED_H
#define LED_H

#define LED_TOGGLE_PERIOD            500
#define LED_TOGGLE_PERIOD_FAILSAFE   1000

#include <Arduino.h>

class Light
{
public:
    Light(int pin_forward, int pin_reversed, int pin_brake, int pin_left, int pin_right, int pin_beacon);
    void begin();

    void ForwardOn();
    void ForwardOff();

    void ReversedOn();
    void ReversedOff();

    void LeftLightOff();
    void RightLightOff();

    void LightSignalOn();
    void LightSignalOff();

    void BeaconOff();

    void BrakeActive(uint16_t time_period, uint16_t constant_time);

    void LeftSignal(uint16_t time_period, uint16_t constant_time);
    void RightSignal(uint16_t time_period, uint16_t constant_time);

    void BeaconSignal(uint16_t time_period, uint16_t constant_time);
    void HazardSignal(uint16_t time_period, uint16_t constant_time);
private:
    int _fpin;
    int _rpin;
    int _bpin;
    int _lspin;
    int _rspin;
    int _bcpin;
};

#endif /* LED_H */