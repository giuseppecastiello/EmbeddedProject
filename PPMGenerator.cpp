#include "PPMGenerator.h"

PPMGenerator::PPMGenerator(int sigPin, int channelNum, int defaultValue) {
  this->sigPin = sigPin;
  this->channelNum = channelNum;
  this->defaultValue = defaultValue;
}

void PPMGenerator::begin() {
  ppm = (int*)malloc(channelNum * sizeof(int));   //memory allocation for control data

  for (int i = 0; i < channelNum; i++) {          //initiallize default control values
    ppm[i] = defaultValue;
  }

  pinMode(sigPin, OUTPUT);                        //set the PPM signal pin as an output
  digitalWrite(sigPin, !ON_STATE);                //set the PPM signal pin to the default state (off)
}

void PPMGenerator::setChan(int chan, int val) {
  ppm[chan] = val;                                //set channel to value usually between 1000 and 2000
}

void PPMGenerator::interruptFunc() {
  for (int i = 0; i < channelNum; i++) {
    digitalWrite(sigPin, ON_STATE);                   //start pulse
    delayMicroseconds(PULSE_LENGTH);                  //wait PULSE_LENGTH
    digitalWrite(sigPin, !ON_STATE);
    delayMicroseconds(ppm[i] - PULSE_LENGTH);//set Compare Match register to transmit channel command
  }
  digitalWrite(sigPin, ON_STATE);                   //start pulse
  delayMicroseconds(PULSE_LENGTH);                  //wait PULSE_LENGTH
  digitalWrite(sigPin, !ON_STATE);
}
