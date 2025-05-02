#ifndef _MPU6500_Kalman_H_
#define _MPU6500_Kalman_H_

#include "Arduino.h"

#define SERIAL_KalmanMPU9250_DEBUG 0
#define RESTRICT_PITCH

class IMU {
public:
  static void init();
  static void read();
  static uint32_t getLastReadTime();
  static int16_t getRawAccelX();
  static int16_t getRawAccelY();
  static int16_t getRawAccelZ();
  static int16_t getRawGyroX();
  static int16_t getRawGyroY();
  static int16_t getRawGyroZ();
  static double getRoll();
  static double getPitch();
  static double getAccelX_g();
  static double getAccelY_g();
  static double getAccelZ_g();
  static double getGyroX_dps();
  static double getGyroY_dps();
  static double getGyroZ_dps();

private:
  static void MPU9250Read();
  static void RollPitchFromAccel(double* roll, double* pitch);

  static uint32_t lastProcessed;
  static int16_t accelX, accelY, accelZ;
  static int16_t gyroX, gyroY, gyroZ;
  static double kalXAngle, kalYAngle;
};

#endif
