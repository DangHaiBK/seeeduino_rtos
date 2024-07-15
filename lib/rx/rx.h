#ifndef RX_H
#define RX_H

#define RX_INPUT_CHANNEL_1 8
#define RX_INPUT_CHANNEL_2 9
#define RX_INPUT_CHANNEL_3 10

#define RX_INPUT_MAP_CHANNEL_1_TO_NUMBER 0
#define RX_INPUT_MAP_CHANNEL_2_TO_NUMBER 1
#define RX_INPUT_MAP_CHANNEL_3_TO_NUMBER 2

#define RECEIVER_PWM_NEUTRAL          1500    /* Typical value */
#define RECEIVER_PWM_MAX              2000    /* Typical value */
#define RECEIVER_PWM_MIN              1000    /* Typical value */
#define RECEIVER_ERROR_RATE           30      /* Typical value +- this value */
#define RECEIVER_FRESH_RATE           100     /* [MS] Frame rate, typical 10 - 300 Hz */
#define RECEIVER_PWM_LOSS_OR_FAIL     255

#define RECEIVER_STICK_MIDDLE         0
#define RECEIVER_STICK_INCREASING     1
#define RECEIVER_STICK_DECREASING     2
#define RECEIVER_STICK_LOSS_OR_FAIL   3

#define RECEIVER_MAX_LENGTH_QUEUES    3

#include <Arduino.h>

/* Class */
class RxReceiver
{
public:
    RxReceiver(uint8_t ch1, uint8_t ch2, uint8_t ch3);
    void begin();
    void rx_read_raw(uint8_t pin, uint8_t channel);
    /* Interrupt functions */
    unsigned long rx_read_pwm(uint8_t channel);
    void rx_read_pulse(uint8_t channel, uint8_t *direct, uint16_t *val);

    void measuring_CH1();
    void measuring_CH2();
    void measuring_CH3();

private:
    /* data */
    uint8_t _ch1;
    uint8_t _ch2;
    uint8_t _ch3;

    /* function */
    void rx_remove_incoming_data_flag(uint8_t channel);
};

/* Functions */
// void measuring_CH1(void);
// void measuring_CH2(void);
// void measuring_CH3(void);

#endif /* RX_H */