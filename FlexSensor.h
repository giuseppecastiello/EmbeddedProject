#ifndef FLEXSENSOR_H
#define FLEXSENSOR_H

#include <arduino.h>

class FlexSensor {
  private:
    int analogPin;              //the pin with ADC on which the sensor is connected

  public:
    FlexSensor(int analogPin);
    void begin();
    int getFlexion();
};

#endif
