#include <Arduino.h>
#include <FreeRTOS_SAMD21.h>

#include "main.h"

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
QueueHandle_t xRxQueue[RECEIVER_NUM_CHANNEL];

SemaphoreHandle_t xMutexBrake;

/* Functions for operation */
void vStartupEffect();

/* Functions for scheduler */
void vGetInputRxChannel1(void *pvParameters);
void vGetInputRxChannel2(void *pvParameters);
void vGetInputRxChannel3(void *pvParameters);
void vNormalBrakeLights(void *pvParameters);
void vControlSpeedMotor(void *pvParameters);
void vControlSteeringLights(void *pvParameters);
void vControlAuxLights(void *pvParameters);

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
    rxReceiver.begin(RX_COMPENSATE_ERROR_ENABLE);

    /* Mapping pins for leds */
    rcLight.begin();

    /* Mapping pins for motor */
    motorSpeed.begin();

    /* Using signal light effect when powering battery up */
    vStartupEffect();

    /* Attach interrupt to input pin */
    attachInterrupt(digitalPinToInterrupt(RX_INPUT_CHANNEL_1), measuring_CH1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RX_INPUT_CHANNEL_2), measuring_CH2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RX_INPUT_CHANNEL_3), measuring_CH3, CHANGE);

    /* Create queues */
    for (uint8_t i=0; i<RECEIVER_NUM_CHANNEL; i++)
    {
        xRxQueue[i] = xQueueCreate(RECEIVER_MAX_LENGTH_QUEUES, sizeof(uint32_t));
    }

    /* Create binary semaphores */
    xMutexBrake = xSemaphoreCreateBinary();

    /* Create tasks */
    xTaskCreate(vGetInputRxChannel1, 
                "Channel 1", 
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL
    );
    
    xTaskCreate(vGetInputRxChannel2, 
                "Channel 2", 
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL
    );

    xTaskCreate(vGetInputRxChannel3, 
                "Channel 3", 
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL
    );
    
    xTaskCreate(vNormalBrakeLights,
                "Normal light",
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL

    );

    xTaskCreate(vControlSpeedMotor,
                "Control Motor",
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL
    );

    xTaskCreate(vControlSteeringLights,
                "Control Steering",
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL
    );

    xTaskCreate(vControlAuxLights,
                "Control Lights",
                128,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL
    );

    /* Give the semaphore at beginning */
    xSemaphoreGive(xMutexBrake);

    /* Start scheduling */
    vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

/*
    * Signal light effect when powering up
*/
void vStartupEffect()
{
    for (uint8_t i=0; i<2; i++)
    {
        rcLight.LightSignalOn();
        delay(500);
        rcLight.LightSignalOff();
        delay(500);
    }
    delay(1000);
}

/* 
    * Clear all data (remaining) in a queue
*/
void vClearQueue(QueueHandle_t xQueue) 
{
    uint32_t dummyData = 0;
    while (xQueueReceive(xQueue, &dummyData, 0) == pdPASS) {}
}

/*
    * Get PWM data from channel 1
    * Put 3 PWM consecutive data into a queue
*/
void vGetInputRxChannel1(void *pvParameters)
{
  (void) pvParameters;
  rxData sRxData, sRxDataSendQueue;
  uint16_t lastPwmValue = 0;
  uint8_t lastStateValue = 0;
  uint16_t zeroValCount = 0;

  for ( ;; )
  {
      rxReceiver.rx_read_pulse(RX_CHANNEL_1, &sRxData.direction, &sRxData.pwmVal);
      if (sRxData.pwmVal != RECEIVER_PWM_LOSS_OR_FAIL)
      {
          lastPwmValue = sRxData.pwmVal;
          lastStateValue = sRxData.direction;
          zeroValCount = 0;
      }
      else 
      {
          zeroValCount ++;
      }
      if (zeroValCount >= RECEIVER_PWM_FAIL_COUNT)
      {
          lastPwmValue = RECEIVER_PWM_LOSS_OR_FAIL;
          lastStateValue = RECEIVER_STICK_LOSS_OR_FAIL;
      }

      sRxDataSendQueue.pwmVal = lastPwmValue;
      sRxDataSendQueue.direction = lastStateValue;
      xQueueSendToBack(xRxQueue[RX_CHANNEL_1], &sRxDataSendQueue, pdMS_TO_TICKS(DELAY_RX_TASK));

      /* Delay before coming back to this task */
      vTaskDelay(pdMS_TO_TICKS(DELAY_RX_TASK));
  }
}

/*
    * Get PWM data from channel 2
    * Put 3 PWM consecutive data into a queue
*/
void vGetInputRxChannel2(void *pvParameters)
{
  (void) pvParameters;
  rxData sRxData, sRxDataSendQueue;
  uint16_t lastPwmValue = 0;
  uint8_t lastStateValue = 0;
  uint16_t zeroValCount = 0;

  for ( ;; )
  {
      rxReceiver.rx_read_pulse(RX_CHANNEL_2, &sRxData.direction, &sRxData.pwmVal);
      if (sRxData.pwmVal != RECEIVER_PWM_LOSS_OR_FAIL)
      {
          lastPwmValue = sRxData.pwmVal;
          lastStateValue = sRxData.direction;
          zeroValCount = 0;
      }
      else 
      {
          zeroValCount ++;
      }
      if (zeroValCount >= RECEIVER_PWM_FAIL_COUNT)
      {
          lastPwmValue = RECEIVER_PWM_LOSS_OR_FAIL;
          lastStateValue = RECEIVER_STICK_LOSS_OR_FAIL;
      }

      sRxDataSendQueue.pwmVal = lastPwmValue;
      sRxDataSendQueue.direction = lastStateValue;
      xQueueSendToBack(xRxQueue[RX_CHANNEL_2], &sRxDataSendQueue, pdMS_TO_TICKS(DELAY_RX_TASK));

      /* Delay before coming back to this task */
      vTaskDelay(pdMS_TO_TICKS(DELAY_RX_TASK));
  }
}

/*
    * Get PWM data from channel 3
    * Put 3 PWM consecutive data into a queue
*/
void vGetInputRxChannel3(void *pvParameters)
{
  (void) pvParameters;
  rxData sRxData, sRxDataSendQueue;
  uint16_t lastPwmValue = 0;
  uint8_t lastStateValue = 0;
  uint16_t zeroValCount = 0;

  for ( ;; )
  {
      rxReceiver.rx_read_pulse(RX_CHANNEL_3, &sRxData.direction, &sRxData.pwmVal);
      if (sRxData.pwmVal != RECEIVER_PWM_LOSS_OR_FAIL)
      {
          lastPwmValue = sRxData.pwmVal;
          lastStateValue = sRxData.direction;
          zeroValCount = 0;
      }
      else 
      {
          zeroValCount ++;
      }
      if (zeroValCount >= RECEIVER_PWM_FAIL_COUNT)
      {
          lastPwmValue = RECEIVER_PWM_LOSS_OR_FAIL;
          lastStateValue = RECEIVER_STICK_LOSS_OR_FAIL;
      }

      sRxDataSendQueue.pwmVal = lastPwmValue;
      sRxDataSendQueue.direction = lastStateValue;
      xQueueSendToBack(xRxQueue[RX_CHANNEL_3], &sRxDataSendQueue, pdMS_TO_TICKS(DELAY_RX_TASK));

      /* Delay before coming back to this task */
      vTaskDelay(pdMS_TO_TICKS(DELAY_RX_TASK));
  }
}

/* 
    * This function controls brake light in normal mode
*/
void vNormalBrakeLights(void *pvParameters)
{
    (void) pvParameters;
    if (xSemaphoreTake(xMutexBrake, pdMS_TO_TICKS(DELAY_MAIN_TASK)) == pdTRUE)
    {
        rcLight.BrakeOnPWM(60);
        xSemaphoreGive(xMutexBrake);
    }
    vTaskDelay(pdMS_TO_TICKS(DELAY_RX_TASK));
}

/* 
    * This function receives data from channel 2 (TX MC6C) to control speed motor
    * with controlling led front, back, and brake
*/
void vControlSpeedMotor(void *pvParameters)
{
    (void) pvParameters;
    rxData sRxData;
    bool brake_status = true;
    uint16_t convertVal = 0;
    uint8_t period = 0;
    uint16_t pwmVal = 0;
    for ( ;; )
    {
        if (xQueueReceive(xRxQueue[RX_CHANNEL_1], &sRxData, pdMS_TO_TICKS(DELAY_MAIN_TASK)) == pdPASS)
        {
        #if (SERIAL_DEBUG == 1)
            //Serial.println("PWM value from Channel 2: ");
            //period = sRxData.direction;
        #endif
            pwmVal = sRxData.pwmVal;     // Get data into a queue
            switch (sRxData.direction)
            {
            /* Loss or failed signal */
            case RECEIVER_STICK_LOSS_OR_FAIL:
                period = 0;
                brake_status = false;

                rcLight.BrakeOff();
                motorSpeed.SetStop();
                break;

            /* Forward */
            case RECEIVER_STICK_INCREASING:
                period = 0;
                brake_status = false;

                rcLight.BrakeOff();

                if (pwmVal >= RECEIVER_PWM_MAX) {
                    motorSpeed.SetFullForward();
                }
                else {
                    convertVal = map(pwmVal, RECEIVER_PWM_NEUTRAL, RECEIVER_PWM_MAX, \
                                        MINIMUM_MOTOR_SPEED, MAXIMUM_MOTOR_SPEED_FORWARD);
                    motorSpeed.SetPwmForward(convertVal);
                }
                break;

            /* Reversed */
            case RECEIVER_STICK_DECREASING:
                period = 0;
                brake_status = false;

                rcLight.BrakeOff();

                if (pwmVal <= RECEIVER_PWM_MIN) {
                    motorSpeed.SetFullReversed();
                }
                else {
                    convertVal = map(pwmVal, RECEIVER_PWM_MIN, RECEIVER_PWM_NEUTRAL, \
                                        MINIMUM_MOTOR_SPEED, MAXIMUM_MOTOR_SPEED_REVERSED);
                    motorSpeed.SetPwmReversed(convertVal);
                }
                break;
            
            /* Parking */
            default:
                motorSpeed.SetStop();
                
                if (brake_status == false) {
                    if (xSemaphoreTake(xMutexBrake, pdMS_TO_TICKS(DELAY_MAIN_TASK)) == pdTRUE) {
                        period ++;
                        rcLight.BrakeOnPWM(100);

                        if (period > LIGHT_BRAKE_PERIOD) {
                            brake_status = true;
                            rcLight.BrakeOnPWM(0);
                            xSemaphoreGive(xMutexBrake);
                        }
                    }
                }
                break;
            }
        }
    }
}

/* 
    * This function receives data from channel 4 (TX MC6C) to control signal lights
    * with controlling led signal left and right
*/
void vControlSteeringLights(void *pvParameters)
{
    (void) pvParameters;
    rxData sRxData;
    uint8_t periodFailsafe = 0;
    uint8_t timeCount = 0;

    for (;;)
    {
        if (xQueueReceive(xRxQueue[RX_CHANNEL_2], &sRxData, pdMS_TO_TICKS(DELAY_MAIN_TASK)) == pdPASS)
        {
            switch (sRxData.direction)
            {
            case RECEIVER_STICK_LOSS_OR_FAIL:
                periodFailsafe ++;
                
                if (periodFailsafe % LIGHT_PERIOD_FAILSAFE == 0) {
                    rcLight.HazardSignal();
                }
                break;

            case RECEIVER_STICK_INCREASING:
                periodFailsafe = 0;
                timeCount ++;

                if (timeCount >= TIME_THRES_STEERING) {
                    if (timeCount % LIGHT_PERIOD == 0) {
                        rcLight.LeftLightToggle();
                    }
                }
                break;
            
            case RECEIVER_STICK_DECREASING:
                periodFailsafe = 0;
                timeCount ++;

                if (timeCount > TIME_THRES_STEERING) {
                    if (timeCount % LIGHT_PERIOD == 0) {
                        rcLight.RightLightToggle();
                    }
                }
                break;
            
            default:                    // Neutral position
                periodFailsafe = 0;
                timeCount = 0;

                rcLight.LightSignalOff();
                break;
            }
        }
    }

}

/* 
    * This function receives data from channel 5 (or 6) (TX MC6C) to control aux lights
    * with controlling aux lights
*/
void vControlAuxLights(void *pvParameters)
{
    (void) pvParameters;
    rxData sRxData;
    uint8_t periodFailSafe = 0;
    uint8_t periodDown = 0;
    uint8_t lastState = 0;
    uint8_t count = 0;

    for (;;)
    {
        if (xQueueReceive(xRxQueue[RX_CHANNEL_3], &sRxData, pdMS_TO_TICKS(DELAY_MAIN_TASK)) == pdPASS)
        {
            switch (sRxData.direction)
            {
            case RECEIVER_STICK_LOSS_OR_FAIL:
                periodFailSafe ++;

                if (periodFailSafe % LIGHT_PERIOD_FAILSAFE == 0) {
                    rcLight.BeaconSignal();
                }

                lastState = RECEIVER_STICK_LOSS_OR_FAIL;
                break;
            
            case RECEIVER_STICK_INCREASING:
                periodFailSafe = 0;

                rcLight.ForwardPWM(100);

                lastState = RECEIVER_STICK_INCREASING;
                break;
            
            case RECEIVER_STICK_DECREASING:
                periodFailSafe = 0;

                if (lastState == RECEIVER_STICK_MIDDLE) {
                    count ++;
                }
                if (count != 0) {  
                    if (count % 2 == 0) {
                        periodDown ++;
                        if (periodDown % LIGHT_PERIOD == 0) {
                            rcLight.BeaconSignal();
                        }
                    }
                    else {
                        rcLight.BeaconOff();
                    }
                }

                lastState = RECEIVER_STICK_DECREASING;
                break;
            
            default:            // Middle position
                periodFailSafe = 0;

                if (lastState == RECEIVER_STICK_LOSS_OR_FAIL) {
                    rcLight.BeaconOff();
                }
                rcLight.ForwardPWM(60);

                lastState = RECEIVER_STICK_MIDDLE;
                break;
            }
        }
    }
}
