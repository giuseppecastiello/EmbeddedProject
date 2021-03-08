#ifndef MPU9250_H
#define MPU9250_H

#include <arduino.h>
#include <Wire.h>

/* --------------- MPU9250 --------------- */
#define MPU9250_ADDR 0x68                                       //MPU9250 I2C address without pulldown wire
#define MPU9250_POWER_MANAG 0x6B                                //MPU9250 I2C power mangement register address
#define MPU9250_ACC_CONFIG 0x1C                                 //MPU9250 I2C accelerometer config register address
#define MPU9250_ACC_DATA 0x3B                                   //MPU9250 I2C accelerometer data register address
#define MPU9250_GYRO_CONFIG 0x1B                                //MPU9250 I2C gyroscope config register address
#define MPU9250_GYRO_DATA 0x43                                  //MPU9250 I2C gyroscope data register address
#define MPU9250_I2C_MASTER_ENABLE 0x6A                          //MPU9250 master I2C enabler register address
#define MPU9250_INTERFACE_BYPASS_MUX_ENABLE 0x37                //MPU9250 I2C bypass mux enabler register address

/* ----- Magnetometer within MPU9250 ----- */
#define AK8963_ADDR 0x0C                                        //AK8963 I2C address
#define AK8963_CTRL_REG_1 0x0A                                  //AK8963 I2C control register address
#define AK8963_STATUS_REG_1 0x02                                //AK8963 I2C status register address
#define AK8963_FUSE_ROM 0x10                                    //AK8963 I2C fuse ROM address
#define AK8963_DATA_READY_MASK 0b00000001                       //magnetometer data ready mask
#define AK8963_OVERFLOW_MASK 0b00001000                         //magnetometer overflow mask


class MPU9250 {
  private:
    int ledPin;                                                 //the pin that drives the LED that indicates status

    float accX, accY, accZ;                                     //raw accelrometer readings
    float accOffsetX = 0, accOffsetY = 0;                       //offsets compensating initial tilt
    float accAngleX, accAngleY;                                 //board angles from accelerometer

    float gyroX, gyroY, gyroZ;                                  //raw accelrometer readings
    float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;    //offsets compensating initial tilt
    float currentTime = 0, previousTime;
    float gyroAngleX, gyroAngleY;                               //board angles from gyroscope

    float angleX = 0, angleY = 0;                               //board angles from complementary filter (gyro+acc)

    int magX, magY, magZ;                                       //raw magnetometer readings
    int magXOffset = 151, magYOffset = -9, magZOffset = -246;   //hard-iron offsets from calibration program
    float magXScale = 1.01, magYScale = 1.01, magZScale = 0.99; //soft-iron scale factors from calibration program
    float ASAX = 1.19, ASAY = 1.19, ASAZ = 1.15;                //Asahi Sensitivity Adjustment fuse ROM values
    float magXCompDampened, magYCompDampened;                   //mag readings compensated from tilt and smoothed

    float angleZ;                                               //board angle from magnetometer
    float angleZOffset = 0;                                     //angle offset compensating initial tilt

  public:
    MPU9250(int ledPin);
    void begin();
    void configureAcc();
    void configureGyro();
    void configureMag();
    void readAcc();
    void readGyro();
    void readMag();
    void calibrateAcc();
    void calibrateGyro();
    void calibrateMag();
    void update();                                              //read acc, gyro, mag and update data
    float getAngleX();
    float getAngleY();
    float getAngleZ();
};

#endif
