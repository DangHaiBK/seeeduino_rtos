#include "led.h"

/* 
    * Call constructor to assign pins  
*/
Light::Light(int pin_forward, int pin_reversed, int pin_brake, int pin_left, int pin_right, int pin_beacon)
{
    _fpin = pin_forward;
    _rpin = pin_reversed;
    _bpin = pin_brake;
    _lspin = pin_left;
    _rspin = pin_right;
    _bcpin = pin_beacon;
}

/* 
    * Initialize pins used for leds
*/
void Light::begin()
{
    pinMode(_fpin, OUTPUT);
    pinMode(_rpin, OUTPUT);
    pinMode(_bpin, OUTPUT);
    pinMode(_lspin, OUTPUT);
    pinMode(_rspin, OUTPUT);
    pinMode(_bcpin, OUTPUT);
}

/* 
    * Turn on led forward 
*/
void Light::ForwardOn()
{
    digitalWrite(_fpin, HIGH);
}

/* 
    * Turn off led forward 
*/
void Light::ForwardOff()
{
    digitalWrite(_fpin, LOW);
}

/*
    * Adjust led forward based on percentage 
*/
void Light::ForwardPWM(uint8_t percentage)
{
    if (percentage >= 100) {
        digitalWrite(_fpin, HIGH);
    }
    else if (percentage <= 0) {
        digitalWrite(_fpin, LOW);
    }
    else {
        analogWrite(_fpin, (uint32_t)((percentage * 255) / 100));
    }
}

/* 
    * Turn on led reversed
*/
void Light::ReversedOn()
{
    digitalWrite(_rpin, HIGH);
}

/* 
    * Turn off led reversed
*/
void Light::ReversedOff()
{
    digitalWrite(_rpin, LOW);
}

/*
    * Turn off led left
*/
void Light::LeftLightOff()
{
    digitalWrite(_lspin, LOW);
}

/*
    * Turn off led right
*/
void Light::RightLightOff()
{
    digitalWrite(_rspin, LOW);
}

/*
    * Toggle led left
*/
void Light::LeftLightToggle()
{
    digitalWrite(_lspin, !digitalRead(_lspin));
}

void Light::RightLightToggle()
{
    digitalWrite(_rspin, !digitalRead(_rspin));
}

void Light::LightSignalOn()
{
    digitalWrite(_lspin, HIGH);
    digitalWrite(_rspin, HIGH);
}

void Light::LightSignalOff()
{
    digitalWrite(_lspin, LOW);
    digitalWrite(_rspin, LOW);
}

/* 
    * Turn off led beacon
*/
void Light::BeaconOff()
{
    digitalWrite(_bcpin, LOW);
}

void Light::BrakeOnPWM(uint8_t percentage)
{
    if (percentage >= 100) {
        digitalWrite(_bpin, HIGH);
    }
    else if (percentage <= 0) {
        digitalWrite(_bpin, LOW);
    }
    else {
        analogWrite(_bpin, (uint32_t)((percentage * 255) / 100));
    }
}

/*
    * Turn on led brake
*/
void Light::BrakeOn()
{
    digitalWrite(_bpin, HIGH);
}

/*
    * Turn off led brake
*/
void Light::BrakeOff()
{
    digitalWrite(_bpin, LOW);
}

/* 
    * Blink led beacon
*/
void Light::BeaconSignal()
{
    digitalWrite(_bcpin, !digitalRead(_bcpin));
}

/*
    * Toggle led hazard (two led signals)
*/
void Light::HazardSignal()
{
    static uint8_t ledState = 0;
    ledState = (ledState == HIGH) ? LOW : HIGH;

    digitalWrite(_lspin, ledState);
    digitalWrite(_rspin, ledState);
}