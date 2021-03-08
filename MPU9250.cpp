#include "MPU9250.h"

MPU9250::MPU9250(int ledPin) {
  this->ledPin = ledPin;
}

void MPU9250::begin() {
  pinMode(ledPin, OUTPUT);                    //set the LED status pin as an output
  digitalWrite(ledPin, LOW);                  //set the LED status pin in the off state

  Wire.begin();                               //initialize I2C communication

  Wire.beginTransmission(MPU9250_ADDR);       //start communication with MPU9250
  Wire.write(MPU9250_POWER_MANAG);            //talk to power management register
  Wire.write(0x00);                           //use internal 20MHz clock
  Wire.endTransmission(true);                 //end the transmission

  configureAcc();
  configureGyro();
  configureMag();

  calibrateAcc();
  calibrateGyro();
  calibrateMag();
}

void MPU9250::configureAcc() {
  Wire.beginTransmission(MPU9250_ADDR);       //start communication with MPU9250
  Wire.write(MPU9250_ACC_CONFIG);             //talk to the accEL_CONFIG register
  Wire.write(0x10);                           //set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);                 //end the transmission
}

void MPU9250::configureGyro() {
  Wire.beginTransmission(MPU9250_ADDR);       //start communication with MPU9250
  Wire.write(MPU9250_GYRO_CONFIG);            //talk to the GYRO_CONFIG register
  Wire.write(0x10);                           //set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);                 //end the transmission
}

void MPU9250::configureMag() {
  /*
     The MPU-9250 contains an AK8963 magnetometer and an
     MPU-9250 gyro/accelerometer within the same package.

     To access the AK8963 magnetometer chip The MPU-9250 I2C bus
     must be changed to pass-through mode. To do this we must:
     - disable the MPU-9250 slave I2C and
     - enable the MPU-9250 interface bypass mux
  */
  Wire.beginTransmission(MPU9250_ADDR);                   //start communication with MPU9250
  Wire.write(MPU9250_I2C_MASTER_ENABLE);                  //point to I2C master interface enabler register
  Wire.write(0x00);                                       //disable the I2C master interface
  Wire.endTransmission();

  // ----- Enable MPU9250 interface bypass mux
  Wire.beginTransmission(MPU9250_ADDR);                   //start communication with MPU9250
  Wire.write(MPU9250_INTERFACE_BYPASS_MUX_ENABLE);        //point to bypass mux enabler register
  Wire.write(0x02);                                       //enable the bypass mux
  Wire.endTransmission();

  /*
     The factory sensitivity readings for the XYZ axes are stored in a fuse ROM.
     To access this data we must change the AK9863 operating mode.
  */
  Wire.beginTransmission(AK8963_ADDR);                    //start communication with AK8963
  Wire.write(AK8963_CTRL_REG_1);                          //point to mode control register 1
  Wire.write(0b00011111);                                 //set mode to enable access to fuse ROM
  Wire.endTransmission();
  delay(100);                                             //wait for mode change

  /*
     Get factory XYZ sensitivity adjustment values from fuse ROM according to datasheet formulas
  */
  Wire.beginTransmission(AK8963_ADDR);                    //start communication with AK8963
  Wire.write(AK8963_FUSE_ROM);                            //point to AK8963 fuse ROM
  Wire.endTransmission();
  Wire.requestFrom(AK8963_ADDR, 3);                       //request 6 byte total
  while (Wire.available() < 3);                           //wait for the data
  ASAX = (Wire.read() - 128) * 0.5 / 128 + 1;             //correct raw data
  ASAY = (Wire.read() - 128) * 0.5 / 128 + 1;
  ASAZ = (Wire.read() - 128) * 0.5 / 128 + 1;

  /*
     Power down AK8963 in order to change mode
     This wasn't necessary for the first mode change as the chip was already powered down
  */
  Wire.beginTransmission(AK8963_ADDR);                    //start communication with AK8963
  Wire.write(AK8963_CTRL_REG_1);                          //point to mode control register 1
  Wire.write(0b00000000);                                 //set mode to power down
  Wire.endTransmission();
  delay(100);                                             //wait for mode change

  // ----- Set output to mode 2 (16-bit, 100Hz continuous)
  Wire.beginTransmission(AK8963_ADDR);                    //start communication with AK8963
  Wire.write(AK8963_CTRL_REG_1);                          //point to mode control register 1
  Wire.write(0b00010110);                                 //set output width to 16bits and measurements frequency to 100Hz
  Wire.endTransmission();
  delay(100);                                             //wait for mode change
}

void MPU9250::readAcc () {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU9250_ACC_DATA);                           //point to accelerometer data register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6, true);                //request 6 byte total, each axis value is stored in 2 byte
  accX = (Wire.read() << 8 | Wire.read()) / 4096.0;       //combine LSB and MSB
  accY = (Wire.read() << 8 | Wire.read()) / 4096.0;       //and according to datasheet for +-8g range divide raw value by 4096
  accZ = (Wire.read() << 8 | Wire.read()) / 4096.0;
}

void MPU9250::readGyro() {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(MPU9250_GYRO_DATA);                          //point to gyroscope data register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6, true);                //request 6 byte total, each axis value is stored in 2 byte
  gyroX = (Wire.read() << 8 | Wire.read()) / 32.8;        //combine LSB and MSB
  gyroY = (Wire.read() << 8 | Wire.read()) / 32.8;        //according to datasheet for 1000dps range divide raw value by 32.8
  gyroZ = (Wire.read() << 8 | Wire.read()) / 32.8;
}

void MPU9250::readMag() {
  int magXLocal, magYLocal, magZLocal;

  Wire.beginTransmission(AK8963_ADDR);                    //start communication with AK8963
  Wire.write(AK8963_STATUS_REG_1);                        //point to status register 1
  Wire.endTransmission();
  Wire.requestFrom(AK8963_ADDR, 1);                       //request 1 data byte
  while (Wire.available() < 1);                           //wait for the data
  if (Wire.read() & AK8963_DATA_READY_MASK) {             //check data ready bit
    Wire.requestFrom(AK8963_ADDR, 7);                     //request 7 byte total, each axis value is stored in 2 byte
    while (Wire.available() < 7);                         //wait for the data
    magXLocal = (Wire.read() | Wire.read() << 8) * ASAX;  //combine LSB and MSB and apply ASA corrections
    magYLocal = (Wire.read() | Wire.read() << 8) * ASAY;
    magZLocal = (Wire.read() | Wire.read() << 8) * ASAZ;
    int status_reg_2 = Wire.read();                       //from last byte requested read status data read

    if (!(status_reg_2 & AK8963_OVERFLOW_MASK)) {         //validate data
      magX = (magXLocal - magXOffset) * magXScale;
      magY = (magYLocal - magYOffset) * magYScale;
      magZ = (magZLocal - magZOffset) * magZScale;
    }
  }
}

void MPU9250::calibrateAcc() {
  digitalWrite(ledPin, HIGH);                             //turn LED on to indicate calibration in process
  delay(300);

  for (int i = 0; i < 500; i++) {                         //read accelerometer values 500 times
    readAcc();
    accOffsetX = accOffsetX + ((atan(accY / sqrt(pow((accX), 2) + pow((accZ), 2))) * 180 / PI));  // Sum all readings
    accOffsetY = accOffsetY + ((atan(-1 * accX / sqrt(pow((accY), 2) + pow((accZ), 2))) * 180 / PI));
  }

  accOffsetX = accOffsetX / 500;                          //divide the sum by 500 to get the offset value
  accOffsetY = accOffsetY / 500;

  digitalWrite(ledPin, LOW);                              //turn LED off
}

void MPU9250::calibrateGyro() {
  digitalWrite(ledPin, HIGH);                             //turn LED on to indicate calibration in process
  delay(300);

  for (int i = 0; i < 500; i++) {                         //read gyroscope values 500 times
    readGyro();
    gyroOffsetX = gyroOffsetX + gyroX;                    //sum all readings
    gyroOffsetY = gyroOffsetY + gyroY;
    gyroOffsetZ = gyroOffsetZ + gyroZ;
  }

  gyroOffsetX = gyroOffsetX / 500;                        //divide the sum by 500 to get the offset value
  gyroOffsetY = gyroOffsetY / 500;
  gyroOffsetZ = gyroOffsetZ / 500;

  digitalWrite(ledPin, LOW);                              //turn LED off
}

void MPU9250::calibrateMag() {
  digitalWrite(ledPin, HIGH);                             //turn LED on to indicate calibration in process
  delay(300);

  for (int i = 0; i < 500; i++) {                         //read magnetometer values 500 times
    readMag();

    /*
      MPU-9250 gyro and AK8963 magnetometer XY axes are orientated 90 degrees to each other which
      means that magPitch equates to the Gyro_roll and magRoll equates to the Gryro_pitch

      The MPU-9520 and AK8963 Z axes point in opposite directions which means
      that the sign for magPitch and magRoll must be negative to compensate.
    */
    float magPitch = -angleX * DEG_TO_RAD;
    float magRoll = -angleY * DEG_TO_RAD;

    float magXCompensated = magX * cos(magPitch) + magY * sin(magRoll) * sin(magPitch) - magZ * cos(magRoll) * sin(magPitch);
    float magYCompensated = magY * cos(magRoll) + magZ * sin(magRoll);    //standart tilt formulas

    magXCompDampened = magXCompDampened * 0.8 + magXCompensated * 0.2;    //dampening data fluctuations
    magYCompDampened = magYCompDampened * 0.8 + magYCompensated * 0.2;

    angleZ = atan2(magXCompDampened, magYCompDampened) * RAD_TO_DEG;  //calculate the angleZ relative to Magnetic North

    angleZOffset += angleZ;                               //sum all readings
  }
  
  angleZOffset = angleZOffset / 500;                      //divide the sum by 500 to get the error value

  digitalWrite(ledPin, LOW);                              //turn LED off
}

void MPU9250::update() {
  readAcc();

  accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI) - accOffsetX; //angle values using acc data
  accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) - accOffsetY;

  previousTime = currentTime;                                       //previous time is stored before the actual time read
  currentTime = millis();                                           //current time is then stored
  float elapsedTime = (currentTime - previousTime) / 1000;          //divide by 1000 to get seconds

  readGyro();

  gyroAngleX = (gyroX - gyroOffsetX) * elapsedTime;                 //raw gyro values are in deg/s
  gyroAngleY = (gyroY - gyroOffsetY) * elapsedTime;                 //multiply by s to get the angle in degrees


  angleX = 0.98 * (angleX + gyroAngleX) + 0.02 * accAngleX;         //complementary filter combining acc and gyro angle values
  angleY = 0.98 * (angleY + gyroAngleY) + 0.02 * accAngleY;

  readMag();

  /*
    MPU-9250 gyro and AK8963 magnetometer XY axes are orientated 90 degrees to each other which
    means that magPitch equates to the Gyro_roll and magRoll equates to the Gryro_pitch

    The MPU-9520 and AK8963 Z axes point in opposite directions which means
    that the sign for magPitch and magRoll must be negative to compensate.
  */
  float magPitch = -angleX * DEG_TO_RAD;
  float magRoll = -angleY * DEG_TO_RAD;

  float magXCompensated = magX * cos(magPitch) + magY * sin(magRoll) * sin(magPitch) - magZ * cos(magRoll) * sin(magPitch);
  float magYCompensated = magY * cos(magRoll) + magZ * sin(magRoll);    //standart tilt formulas

  magXCompDampened = magXCompDampened * 0.8 + magXCompensated * 0.2;    //dampening data fluctuations
  magYCompDampened = magYCompDampened * 0.8 + magYCompensated * 0.2;

  angleZ = atan2(magXCompDampened, magYCompDampened) * RAD_TO_DEG;      //calculate the angleZ relative to Magnetic North
}

float MPU9250::getAngleX() {
  return angleX;
}

float MPU9250::getAngleY() {
  return angleY;
}

float MPU9250::getAngleZ() {
  return angleZ - angleZOffset;
}
