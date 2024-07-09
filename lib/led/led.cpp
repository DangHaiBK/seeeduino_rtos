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

void Light::LeftLightOff()
{
    digitalWrite(_lspin, LOW);
}

void Light::RightLightOff()
{
    digitalWrite(_rspin, LOW);
}

void Light::BeaconOff()
{
    digitalWrite(_bcpin, LOW);
}

/* 
    * Blink led signal left
    * Params:
    * - time_period: toggle led time
*/
void Light::LeftSignal(uint16_t time_period, uint16_t constant_time)
{
    digitalWrite(_rspin, LOW);   // Turn off led right signal
    if (time_period % constant_time == 0)  // Toggle led left and beacon left every 500 ms
    {
        /* Check the previous status of left led */
        if (digitalRead(_lspin) == LOW) {
            digitalWrite(_lspin, HIGH);
        }
        else {
            digitalWrite(_lspin, LOW);
        }
    }
}

/* 
    * Blink led signal right
    * Params:
    * - time_period: toggle led time
*/
void Light::RightSignal(uint16_t time_period, uint16_t constant_time)
{
    digitalWrite(_lspin, LOW);   // Turn off led right signal
    if (time_period % constant_time == 0)  // Toggle led left and beacon left every 500 ms
    {
        /* Check the previous status of left led */
        if (digitalRead(_rspin) == LOW) {
            digitalWrite(_rspin, HIGH);
        }
        else {
            digitalWrite(_rspin, LOW);
        }
    }
}

/* 
    * Active led brake
    * Params:
    * - time_period: toggle led time
*/
void Light::BrakeActive(uint16_t time_period, uint16_t constant_time)
{
    digitalWrite(_bpin, HIGH);      // On brake led when active
    if (time_period % constant_time == 0)     // After 500ms, off this led
    {
        digitalWrite(_bpin, LOW);
    }
}

/* 
    * Blink led beacon
    * Params:
    * - time_period: toggle led time
*/
void Light::BeaconSignal(uint16_t time_period, uint16_t constant_time)
{
    /* Toggle led every 500ms */
    if (time_period % constant_time == 0)
    {
        if (digitalRead(_bcpin) == LOW) {
            digitalWrite(_bcpin, HIGH);
        }
        else {
            digitalWrite(_bcpin, LOW);
        }
    }
}

void Light::HazardSignal(uint16_t time_period, uint16_t constant_time)
{
    if (time_period % constant_time == 0)
    {
        if (digitalRead(_lspin) == LOW) {
            digitalWrite(_lspin, HIGH);
        }
        else {
            digitalWrite(_lspin, LOW);
        }

        if (digitalRead(_rspin) == LOW) {
            digitalWrite(_rspin, HIGH);
        }
        else {
            digitalWrite(_rspin, LOW);
        }
    }
}