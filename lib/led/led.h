#ifndef LED_H
#define LED_H

#define LED_TOGGLE_PERIOD            1000   // ms
#define LED_TOGGLE_PERIOD_FAILSAFE   300    // ms

#include <Arduino.h>

class Light
{
public:
    Light(int pin_forward, int pin_reversed, int pin_brake, int pin_left, int pin_right, int pin_beacon);
    void begin();

    void ForwardOn();
    void ForwardOff();

    void ForwardPWM(uint8_t percentage);

    void ReversedOn();
    void ReversedOff();

    void LeftLightOff();
    void RightLightOff();

    void LeftLightToggle();
    void RightLightToggle();

    void LightSignalOn();
    void LightSignalOff();

    void BeaconOff();

    void BrakeOnPWM(uint8_t percentage);
    void BrakeOn();
    void BrakeOff();

    void BeaconSignal();
    void HazardSignal();
private:
    int _fpin;
    int _rpin;
    int _bpin;
    int _lspin;
    int _rspin;
    int _bcpin;
};

#endif /* LED_H */