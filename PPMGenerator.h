#ifndef PPMGENERATOR_H
#define PPMGENERATOR_H

#include <arduino.h>

/* -------------------------- CONFIGURATION -------------------------- */
#define FRAME_LENGTH 22500    //set the PPM frame length in microseconds
#define PULSE_LENGTH 300      //set the pulse length
#define ON_STATE 1            //set polarity of the pulses: 1 is positive

class PPMGenerator {
  private:
    int sigPin;               //the pin that outputs the PPM signal
    int channelNum;
    int defaultValue;         //channel's default  value
    int *ppm;                 //array of PPM values ranging 1000-2000

  public:
    PPMGenerator(int sigPin, int channelNum, int defaultValue);
    void begin();
    void setChan(int chan, int val);
    void interruptFunc();
};

#endif
