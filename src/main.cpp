#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>
#include "rx.h"
#include "led.h"
#include "motor.h"
#include "main.h"

#define LIGHT_PERIOD (LED_TOGGLE_PERIOD / RECEIVER_FRESH_RATE)
#define LIGHT_PERIOD_EACH_CHANNEL (LED_TOGGLE_PERIOD_FAILSAFE / RECEIVER_FRESH_RATE)

/* Typedef struct for containing RX data package */
struct rxData
{
  uint8_t direction;
  uint16_t pwmVal;
};

/* Create an instance for class RX receiver */
RxReceiver rxReceiver(RX_INPUT_CHANNEL_1, RX_INPUT_CHANNEL_2, RX_INPUT_CHANNEL_3);

/* Create an instance for class Light */
Light rcLight(1, 2, 3, 4, 5, 6);

/* Create an instance for class Motor */
MotorSpeed motorSpeed(1, 2);

/* Create queues to store PWM values from each channel */
QueueHandle_t xRxQueue[3];

QueueSetHandle_t xRxQueueSet;

SemaphoreHandle_t xMutex;

/* Functions for scheduler */
void vGetInputRxChannel1(void *pvParameters);
void vGetInputRxChannel2(void *pvParameters);
void vGetInputRxChannel3(void *pvParameters);
void vControlSpeedMotor(void *pvParameters);
void vControlAuxAndSignalLights(void *pvParameters);

/* 
    * Functions for interrupt signal from RX Receiver ---------------
*/
void measuring_CH1()
{
    rxReceiver.rx_read_raw(RX_INPUT_CHANNEL_1, RX_INPUT_MAP_CHANNEL_1_TO_NUMBER);
}

void measuring_CH2()
{
    rxReceiver.rx_read_raw(RX_INPUT_CHANNEL_2, RX_INPUT_MAP_CHANNEL_2_TO_NUMBER);
}

void measuring_CH3()
{
    rxReceiver.rx_read_raw(RX_INPUT_CHANNEL_3, RX_INPUT_MAP_CHANNEL_3_TO_NUMBER);
}

void setup() {

#if (SERIAL_DEBUG == 1)
    /* Serial initialization */
    Serial.begin(9600);
#endif
    
    /* Mapping pins for receiver */
    rxReceiver.begin();

    /* Mapping pins for leds */
    rcLight.begin();

    /* Mapping pins for motor */
    motorSpeed.begin();

    /* Attach interrupt to input pin */
    attachInterrupt(digitalPinToInterrupt(RX_INPUT_CHANNEL_1), measuring_CH1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RX_INPUT_CHANNEL_2), measuring_CH2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RX_INPUT_CHANNEL_3), measuring_CH3, CHANGE);

    /* Create queues */
    for (uint8_t i=0; i<3; i++)
    {
        xRxQueue[i] = xQueueCreate(6, sizeof(uint32_t));
    }

    /* Create a queue set to contain up to 2 queues */
    xRxQueueSet = xQueueCreateSet(6 * 2);

    if (xRxQueueSet != NULL)
    {
        xQueueAddToSet(xRxQueue[1], xRxQueueSet);
        xQueueAddToSet(xRxQueue[2], xRxQueueSet);
    }

    /* Create a mutex */
    xMutex = xSemaphoreCreateMutex();

    /* Create tasks */
    xTaskCreate(vGetInputRxChannel1, 
                "Channel 1", 
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
    
    xTaskCreate(vGetInputRxChannel2, 
                "Channel 2", 
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(vGetInputRxChannel3, 
                "Channel 3", 
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(vControlSpeedMotor,
                "Control Motor",
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(vControlAuxAndSignalLights,
                "Control Lights",
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);
    
    /* Start scheduling */
    vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

void vGetInputRxChannel1(void *pvParameters)
{
  (void) pvParameters;
  rxData sRxData;
  for ( ;; )
  {
      rxReceiver.rx_read_pulse(0, &sRxData.direction, &sRxData.pwmVal);
      xQueueSendToBack(xRxQueue[0], &sRxData, pdMS_TO_TICKS(100));
      vTaskDelay(pdMS_TO_TICKS(100));
  }
  
}

void vGetInputRxChannel2(void *pvParameters)
{
  (void) pvParameters;
  rxData sRxData;
  for ( ;; )
  {
      rxReceiver.rx_read_pulse(1, &sRxData.direction, &sRxData.pwmVal);
      xQueueSendToBack(xRxQueue[1], &sRxData, pdMS_TO_TICKS(100));
      vTaskDelay(pdMS_TO_TICKS(100));
  }
  
}

void vGetInputRxChannel3(void *pvParameters)
{
  (void) pvParameters;
  rxData sRxData;
  for ( ;; )
  {
      rxReceiver.rx_read_pulse(2, &sRxData.direction, &sRxData.pwmVal);
      xQueueSendToBack(xRxQueue[2], &sRxData, pdMS_TO_TICKS(100));
      vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/* 
    * This function receives data from channel 2 (TX MC6C) to control speed motor
    * with controlling led front, back, and brake
*/
void vControlSpeedMotor(void *pvParameters)
{
    (void) pvParameters;
    rxData sRxData;
    uint16_t convertVal;
    uint16_t period = 0; 
    uint16_t periodFailsafe = 0;
    for ( ;; )
    {
        if (xQueueReceive(xRxQueue[0], &sRxData, pdMS_TO_TICKS(100)) == pdPASS)
        {
            /* Loss signal */
            if (sRxData.direction == RECEIVER_STICK_LOSS_OR_FAIL)
            {
                motorSpeed.SetStop();
                periodFailsafe ++;
                rcLight.ForwardOff();
                rcLight.ReversedOff();
                rcLight.BeaconSignal(periodFailsafe, LIGHT_PERIOD);
                rcLight.HazardSignal(periodFailsafe, LIGHT_PERIOD);

            // #if (SERIAL_DEBUG == 1)
            //     Serial.println("Vehicle in failsafe mode");
            // #endif
            }
            /* Forward */
            else if (sRxData.direction == RECEIVER_STICK_INCREASING)
            {
                period = 0;
                periodFailsafe = 0;
                rcLight.ForwardOn();
                rcLight.ReversedOff();
                if (sRxData.pwmVal >= RECEIVER_PWM_MAX)
                {
                    motorSpeed.SetFullForward();
                }
                else 
                {
                    convertVal = map(sRxData.pwmVal, RECEIVER_PWM_NEUTRAL, RECEIVER_PWM_MAX, 0, 230);
                    motorSpeed.SetPwmForward(convertVal);
                }
            }
            /* Reversed */
            else if (sRxData.direction == RECEIVER_STICK_DECREASING)
            {
                period = 0;
                periodFailsafe = 0;
                rcLight.ReversedOn();
                rcLight.ForwardOff();
                if (sRxData.pwmVal <= RECEIVER_PWM_MIN)
                {
                    motorSpeed.SetFullReversed();
                }
                else 
                {
                    convertVal = map(sRxData.pwmVal, RECEIVER_PWM_MIN, RECEIVER_PWM_NEUTRAL, 0, 127);
                    motorSpeed.SetPwmReversed(convertVal);
                }
            }
            /* Parking */
            else 
            {
                motorSpeed.SetStop();
                rcLight.ForwardOff();
                rcLight.ReversedOff();
                period++;
                if (period <= 5)
                { 
                    rcLight.BrakeActive(period, LIGHT_PERIOD);
                }
                else 
                {
                    period = 6;     // Avoid to increment counter (overflow and reset to 0)
                }
            }
        }
    }
}

/*
    * This function is ultilized to control Light system in the vehicle 
    * Use Channel 4 and Channel 5 (TX MC6C) as Steering channel and Aux channel, respectively
    * Steering channel for signal lights and Aux channel for hazard and beacon lights
*/
void vControlAuxAndSignalLights(void *pvParameters)
{
    (void) pvParameters;
    rxData sRxData;
    uint8_t periodFailsafe = 0;
    uint8_t periodOneDirection = 0;
    uint8_t periodOtherDirection = 0;
    uint8_t modeFromQueue1 = 0;
    uint8_t modeFromQueue2 = 0;
    uint16_t pwmVal = 0;
    QueueHandle_t xQueueContainData;
    
    for ( ;; )
    {
        xQueueContainData = (QueueHandle_t) xQueueSelectFromSet(xRxQueueSet, pdMS_TO_TICKS(200));

        if (xQueueContainData == xRxQueue[1])
        {
            if (xQueueReceive(xRxQueue[1], &sRxData, 0) == pdPASS)
            {
                modeFromQueue1 = sRxData.direction;
                pwmVal = sRxData.pwmVal;
            }
        }
        else if (xQueueContainData == xRxQueue[2])
        {
            if (xQueueReceive(xRxQueue[2], &sRxData, 0) == pdPASS)
            {
                modeFromQueue2 = sRxData.direction;
            }
        }

        /* Both channels fall in failsafe mode, toggle led in 100 millis */
        if ((modeFromQueue1 + modeFromQueue2) == 6)
        {
            periodFailsafe ++;
            periodOneDirection = 0;
            periodOtherDirection = 0;
            rcLight.HazardSignal(periodFailsafe, LIGHT_PERIOD_EACH_CHANNEL);
            rcLight.BeaconSignal(periodFailsafe, LIGHT_PERIOD_EACH_CHANNEL);
            Serial.println("Failsafe ...");
            Serial.println(pwmVal);
        }
        else 
        {
            periodFailsafe = 0;

            /* Total direction = 5 */
            if ((modeFromQueue1 + modeFromQueue2) == 5)
            {
                if ((modeFromQueue1 == 3) && (modeFromQueue2 == 2))   // Failsafe on steering CH, decrease AUX CH
                {
                    periodOneDirection = 0;
                    periodOtherDirection ++;
                    rcLight.BeaconSignal(periodOtherDirection, LIGHT_PERIOD_EACH_CHANNEL);
                    rcLight.LeftLightOff();
                    rcLight.RightLightOff();
                }
                if ((modeFromQueue1 == 2) && (modeFromQueue2 == 3))   // Failsafe on AUX CH, decrease on steering CH
                {
                    periodOneDirection ++;
                    periodOtherDirection = 0;
                    rcLight.BeaconSignal(periodOneDirection, LIGHT_PERIOD_EACH_CHANNEL);
                    rcLight.RightSignal(periodOneDirection, LIGHT_PERIOD);
                }
            }

            /* Total direction = 4 */
            if ((modeFromQueue1 + modeFromQueue2) == 4)
            {
                if ((modeFromQueue1 == 3) && (modeFromQueue2 == 1))   // Failsafe on steering CH, increase AUX CH
                {
                    periodOneDirection ++;
                    rcLight.HazardSignal(periodOneDirection, LIGHT_PERIOD_EACH_CHANNEL);
                }
                if ((modeFromQueue1 == 1) && (modeFromQueue2 == 3))   // Failsafe on AUX CH, increasing steering CH
                {
                    periodOneDirection ++;
                    periodOtherDirection ++;
                    rcLight.BeaconSignal(periodOtherDirection, LIGHT_PERIOD_EACH_CHANNEL);
                    rcLight.LeftSignal(periodOneDirection, LIGHT_PERIOD);
                }
                if ((modeFromQueue1 == 2) && (modeFromQueue2 == 2))   // Both channels decrease
                {
                    periodOneDirection ++;
                    periodOtherDirection ++;
                    rcLight.RightSignal(periodOneDirection, LIGHT_PERIOD);
                    rcLight.BeaconSignal(periodOtherDirection, LIGHT_PERIOD);
                }
            }

            /* Total direction = 3 */
            if ((modeFromQueue1 + modeFromQueue2) == 3)
            {
                /* Failsafe on either channel */
                if ((modeFromQueue1 == 3) && (modeFromQueue2 == 0))   // Steering channel
                {
                    periodOneDirection ++;
                    rcLight.HazardSignal(periodOneDirection, LIGHT_PERIOD_EACH_CHANNEL);
                }
                if ((modeFromQueue1 == 0) && (modeFromQueue2 == 3))   // AUX channel
                {
                    periodOtherDirection ++;
                    rcLight.BeaconSignal(periodOtherDirection, LIGHT_PERIOD_EACH_CHANNEL);
                }
                if ((modeFromQueue1 == 1) && (modeFromQueue2 == 2))   // Increase steering CH, decrease AUX CH
                {
                    periodOneDirection ++;
                    periodOtherDirection ++;
                    rcLight.LeftSignal(periodOneDirection, LIGHT_PERIOD);
                    rcLight.BeaconSignal(periodOtherDirection, LIGHT_PERIOD);
                }
                if ((modeFromQueue1 == 2) && (modeFromQueue2 == 1))   // Decrease steering CH, increase AUX CH
                {
                    periodOneDirection ++;
                    periodOtherDirection ++;
                    rcLight.RightSignal(periodOneDirection, LIGHT_PERIOD);
                }
            }

            /* Total direction = 2 */
            if ((modeFromQueue1 + modeFromQueue2) == 2)
            {
                if ((modeFromQueue1 == 0) && (modeFromQueue2 == 2))   // Steering CH middle, AUX CH decrease
                {
                    periodOneDirection = 0;
                    periodOtherDirection ++;
                    rcLight.LeftLightOff();
                    rcLight.RightLightOff();
                    rcLight.BeaconSignal(periodOtherDirection, LIGHT_PERIOD);
                }
                if ((modeFromQueue1 == 2) && (modeFromQueue2 == 0))   // Steering CH decrease, AUX CH middle
                {
                    periodOneDirection ++;
                    periodOtherDirection = 0;
                    rcLight.RightSignal(periodOneDirection, LIGHT_PERIOD);
                    rcLight.BeaconOff();
                }
                if ((modeFromQueue1 == 1) && (modeFromQueue2 == 1))   // Steering CH increase, AUX CH increase
                {
                    periodOneDirection ++;
                    rcLight.LeftSignal(periodOneDirection, LIGHT_PERIOD);
                }
            }

            /* Total direction = 1 */
            if ((modeFromQueue1 + modeFromQueue2) == 1)
            {
                if ((modeFromQueue1 == 0) && (modeFromQueue2 == 1))   // Steering CH middle, AUX CH increase
                {
                    periodOneDirection = 0;
                    periodOtherDirection ++;
                    rcLight.HazardSignal(periodOtherDirection, LIGHT_PERIOD);
                }
                if ((modeFromQueue1 == 1) && (modeFromQueue2 == 0))   //Steering CH increase, AUX CH middle
                {
                    periodOtherDirection = 0;
                    periodOneDirection ++;
                    rcLight.LeftSignal(periodOneDirection, LIGHT_PERIOD);
                    rcLight.BeaconOff();
                }
            }

            /* Total direction = 0 -> Both channels in middle*/
            if ((modeFromQueue1 + modeFromQueue2) == 0) 
            {
                periodOneDirection = 0;
                periodOtherDirection = 0;

                rcLight.BeaconOff();
                rcLight.LeftLightOff();
                rcLight.RightLightOff();
            }
        }
    }

}