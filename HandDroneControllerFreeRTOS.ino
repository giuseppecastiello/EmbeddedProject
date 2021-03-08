#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <task.h>

#include "MPU9250.h"
#include "PPMGenerator.h"
#include "FlexSensor.h"

/* ----------------------- CONFIGURATION ----------------------- */
#define ANALOG_PIN A2                   //set flex sensor input pin
#define LED_PIN LED_BUILTIN             //set LED status output pin
#define SIG_PIN 10                      //set PPM signal output pin
#define CHANNEL_NUMBER 5                //set the number of channel
#define CHANNEL_DEFAULT_VALUE 1000      //set channel default value

/* 
 * PPM object must be instantiated in the global namespace because the ISR does
 * not support parameters and among them there are the data to be transmitted
 */
PPMGenerator PPM(SIG_PIN, CHANNEL_NUMBER, CHANNEL_DEFAULT_VALUE);
MPU9250 IMU(LED_PIN);                 //instantiate IMU object
FlexSensor flexSensor(ANALOG_PIN);    //instantiate flexSensor object

void setup() {
  Serial.begin(9600);
  Serial.println("Ciao");
  Serial.println(pdMS_TO_TICKS(30));
  
  PPM.begin();                          //configure PPM object
  
  IMU.begin();                          //configure IMU object
  
  flexSensor.begin();                   //configure flexSensor object

  xTaskCreate(taskReadIMU, "readIMU", 128, NULL, 0, NULL);
  xTaskCreate(taskReadFlex, "readFlex", 128, NULL, 0, NULL);
  TimerHandle_t xTimer = xTimerCreate("taskPPMGenerator", pdMS_TO_TICKS(30), pdTRUE, 0, taskPPMGenerator);
  xTimerStart(xTimer, 0);
  
}

void taskReadIMU(void *pvParameters) {
  while(true) {
    Serial.print("b");
    IMU.update();                       //let the IMU update his readings
    float angleX = IMU.getAngleX();     //get readings from IMU
    float angleY = IMU.getAngleY();
    float angleZ = IMU.getAngleZ();
    
    angleX = map(angleX, -90, 90, 1000, 2000);  //map input readings into control data
    angleY = map(angleY, -90, 90, 1000, 2000);
    angleZ = map(angleZ, -180, 180, 1000, 2000);

    angleX = constrain(angleX, 1000, 2000);       //PPM signal range is usually 1000-2000
    angleY = constrain(angleY, 1000, 2000);
    angleZ = constrain(angleZ, 1000, 2000);

    PPM.setChan(0, angleX);             //update channels with data to be transmitted
    PPM.setChan(1, angleY);
    PPM.setChan(3, angleZ);
    taskYIELD();
  }
}

void taskReadFlex(void *pvParameters) {
  while(true) {
    Serial.print("a");
    int throttle = flexSensor.getFlexion();           //get readings flexSensor
    
    throttle = map(throttle, 450, 650, 1000, 2000);   //map input readings into control data
    
    throttle = constrain(throttle, 1000, 2000);       //PPM signal range is usually 1000-2000

    PPM.setChan(2, throttle);                         //update channels with data to be transmitted
    taskYIELD();
  }
}

/*
 * PPM signal needs to be exactly timed, TIMER1 and Interrupt 
 * are so used to genereate it in the background
 */
void taskPPMGenerator(TimerHandle_t xTimer) {
  Serial.println(millis());
  PPM.interruptFunc();
}

void loop() {}
