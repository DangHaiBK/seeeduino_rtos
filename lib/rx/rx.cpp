#include "rx.h"

volatile bool rx_new_pulse[] = {false, false, false};
volatile unsigned long rx_pulse_begin[] = {0, 0, 0};
volatile unsigned long rx_channel_width[] = {0, 0, 0};

/* 
    * Call constructor to assign pins
*/
RxReceiver::RxReceiver(uint8_t ch1, uint8_t ch2, uint8_t ch3)
{
    _ch1 = ch1;
    _ch2 = ch2;
    _ch3 = ch3;
}

/* 
    * Initialize RX pins
*/
void RxReceiver::begin()
{
    pinMode(_ch1, INPUT);
    pinMode(_ch2, INPUT);
    pinMode(_ch3, INPUT);

}

/*
    * Read raw pwm pulse from RX receiver
*/
void RxReceiver::rx_read_raw(uint8_t pin, uint8_t channel)
{
    /* Rising edge */
    if (digitalRead(pin) == HIGH) {
        rx_pulse_begin[channel] = micros();
    }
    /* Falling edge and write the result */
    else {
        rx_channel_width[channel] = micros() - rx_pulse_begin[channel];
        rx_new_pulse[channel] = true;
    }
}

/*
    * Remove the receiving new pulse
*/
void RxReceiver::rx_remove_incoming_data_flag(uint8_t channel)
{
    rx_new_pulse[channel] = false;
}

/* 
    * Read pulse width from RX receiver
*/
unsigned long RxReceiver::rx_read_pwm(uint8_t channel)
{
    //rx_remove_incoming_data_flag(channel);
    rx_new_pulse[channel] = false;
    return rx_channel_width[channel];
}

/* 
    * Decode pulse with pulse width and pulse direction
*/
void RxReceiver::rx_read_pulse(uint8_t channel, uint8_t *direct, uint16_t *val)
{
    unsigned long ulValue = rx_read_pwm(channel);

    if (ulValue > RECEIVER_PWM_NEUTRAL)
    {
       if ((ulValue - RECEIVER_PWM_NEUTRAL) > RECEIVER_ERROR_RATE)
       {
         *direct = RECEIVER_STICK_INCREASING;
         *val = ulValue;
       }
       else 
       {
         *direct = RECEIVER_STICK_MIDDLE;
         *val = RECEIVER_PWM_NEUTRAL;
       }
    }
    //if (ulValue < RECEIVER_PWM_NEUTRAL)
    else
    {
       if ((RECEIVER_PWM_NEUTRAL - ulValue) > RECEIVER_ERROR_RATE)
       {
         *direct = RECEIVER_STICK_DECREASING;
         *val = ulValue;
       }
       else 
       {
         *direct = RECEIVER_STICK_MIDDLE;
         *val = RECEIVER_PWM_NEUTRAL;
       }
    }

    // Uncomment when using normal without failed event detection (upper bound)
//    if (ulValue > RECEIVER_PWM_MAX)
//    {
//      *direct = RECEIVER_STICK_INCREASING;
//      *val = RECEIVER_PWM_MAX;
//    }

    // Adding this code to detect failed event (loss or out of range signal)
    if ((ulValue - RECEIVER_PWM_MAX) > RECEIVER_ERROR_RATE)
    {
      *direct = RECEIVER_STICK_LOSS_OR_FAIL;
      *val = RECEIVER_PWM_LOSS_OR_FAIL;
    }
    else
    {
      if ((RECEIVER_PWM_MAX - ulValue) <= RECEIVER_ERROR_RATE)
      {
        *direct = RECEIVER_STICK_INCREASING;
        *val = RECEIVER_PWM_MAX;
      }
    }
//    else 
//    {
//      if ((RECEIVER_PWM_MAX - ulValue) <= RECEIVER_ERROR_RATE)
//      {
//        *direct = RECEIVER_STICK_INCREASING;
//        *val = RECEIVER_PWM_MAX;
//      }
//    }

    // Uncomment when using normal without failed event detection (lower bound)
//    if (ulValue <= RECEIVER_PWM_MIN)
//    {
//      *direct = RECEIVER_STICK_DECREASING;
//      *val = RECEIVER_PWM_MIN;
//    }
    
    // Adding this code to detect failed event (loss or out of range signal)
    if ((RECEIVER_PWM_MIN - ulValue) > RECEIVER_ERROR_RATE)
    {
      *direct = RECEIVER_STICK_LOSS_OR_FAIL;
      *val = RECEIVER_PWM_LOSS_OR_FAIL;
    }
    else
    {
      if ((ulValue - RECEIVER_PWM_MIN) <= RECEIVER_ERROR_RATE)
      {
        *direct = RECEIVER_STICK_DECREASING;
        *val = RECEIVER_PWM_MIN;
      }
    }
}