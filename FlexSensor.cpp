#include "FlexSensor.h"

FlexSensor::FlexSensor(int analogPin) {
  this->analogPin = analogPin;
}

void FlexSensor::begin() {
  //set flex sensor pin as an input with internal pull-up resistance
  pinMode(analogPin, INPUT_PULLUP);
}

/*
 * Next function returns the value read by the ADC that is the output of
 * the voltage divider made up by flex sensor and Arduino pull-up resistor
 */
int FlexSensor::getFlexion() {
  return analogRead(analogPin);
}
