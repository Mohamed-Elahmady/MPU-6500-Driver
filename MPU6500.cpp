#include "MPU6500.h"
#include <Wire.h>

#if SERIAL_KalmanMPU9250_DEBUG
#define DEBUG_INIT() Serial.begin(115200)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_INIT()
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#ifndef M_PI
#define M_PI 3.14159265359
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 180.0 / M_PI
#endif

#define sqr(x) ((x)*(x))
#define hypotenuse(x, y) sqrt(sqr(x) + sqr(y))

#define IMU_ADDR 0x68
#define IMU_PWR_MGMT_1 0x6B
#define IMU_ACCEL_XOUT_H 0x3B

typedef struct {
  double Q_angle, Q_bias, R_measure;
  double angle, bias, rate;
  double P[2][2], K[2], y, S;
} Kalman;

static Kalman kalmanX, kalmanY;
static double gyroXAngle, gyroYAngle;

uint32_t IMU::lastProcessed = 0;
int16_t IMU::accelX, IMU::accelY, IMU::accelZ;
int16_t IMU::gyroX, IMU::gyroY, IMU::gyroZ;
double IMU::kalXAngle, IMU::kalYAngle;

void Kalman_Init(Kalman* kal) {
  kal->Q_angle = 0.001;
  kal->Q_bias = 0.003;
  kal->R_measure = 0.03;
  kal->angle = kal->bias = 0;
  kal->P[0][0] = kal->P[1][1] = kal->P[0][1] = kal->P[1][0] = 0;
}

double Kalman_GetAngle(Kalman* kal, double newAngle, double newRate, double dt) {
  kal->rate = newRate - kal->bias;
  kal->angle += dt * kal->rate;

  kal->P[0][0] += dt * (dt * kal->P[1][1] - kal->P[0][1] - kal->P[1][0] + kal->Q_angle);
  kal->P[0][1] -= dt * kal->P[1][1];
  kal->P[1][0] -= dt * kal->P[1][1];
  kal->P[1][1] += kal->Q_bias * dt;

  kal->S = kal->P[0][0] + kal->R_measure;
  kal->K[0] = kal->P[0][0] / kal->S;
  kal->K[1] = kal->P[1][0] / kal->S;

  kal->y = newAngle - kal->angle;
  kal->angle += kal->K[0] * kal->y;
  kal->bias += kal->K[1] * kal->y;

  kal->P[0][0] -= kal->K[0] * kal->P[0][0];
  kal->P[0][1] -= kal->K[0] * kal->P[0][1];
  kal->P[1][0] -= kal->K[1] * kal->P[0][0];
  kal->P[1][1] -= kal->K[1] * kal->P[0][1];

  return kal->angle;
}

void IMU::init() {
  DEBUG_INIT();
  Wire.begin();
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(IMU_PWR_MGMT_1);
  Wire.write(0x00);  // Wake up
  Wire.endTransmission(true);
  delay(100);

  Kalman_Init(&kalmanX);
  Kalman_Init(&kalmanY);

  MPU9250Read();
  double roll, pitch;
  RollPitchFromAccel(&roll, &pitch);

  kalmanX.angle = roll;
  kalmanY.angle = pitch;
  gyroXAngle = roll;
  gyroYAngle = pitch;

  lastProcessed = micros();
}

void IMU::read() {
  MPU9250Read();
  double dt = (double)(micros() - lastProcessed) / 1000000.0;
  lastProcessed = micros();

  double roll, pitch;
  RollPitchFromAccel(&roll, &pitch);

  double gyroXRate = (double)gyroX / 131.0;
  double gyroYRate = (double)gyroY / 131.0;

#ifdef RESTRICT_PITCH
  if ((roll < -90 && kalXAngle > 90) || (roll > 90 && kalXAngle < -90)) {
    kalmanX.angle = roll;
    kalXAngle = roll;
    gyroXAngle = roll;
  } else {
    kalXAngle = Kalman_GetAngle(&kalmanX, roll, gyroXRate, dt);
  }

  if (abs(kalXAngle) > 90) gyroYRate = -gyroYRate;
  kalYAngle = Kalman_GetAngle(&kalmanY, pitch, gyroYRate, dt);
#else
  if ((pitch < -90 && kalYAngle > 90) || (pitch > 90 && kalYAngle < -90)) {
    kalmanY.angle = pitch;
    kalYAngle = pitch;
    gyroYAngle = pitch;
  } else {
    kalYAngle = Kalman_GetAngle(&kalmanY, pitch, gyroYRate, dt);
  }

  if (abs(kalYAngle) > 90) gyroXRate = -gyroXRate;
  kalXAngle = Kalman_GetAngle(&kalmanX, roll, gyroXRate, dt);
#endif

  gyroXAngle += gyroXRate * dt;
  gyroYAngle += gyroYRate * dt;

  if (gyroXAngle < -180 || gyroXAngle > 180) gyroXAngle = kalXAngle;
  if (gyroYAngle < -180 || gyroYAngle > 180) gyroYAngle = kalYAngle;
}

void IMU::MPU9250Read() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(IMU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 14, true);

  accelX = (int16_t)(Wire.read() << 8 | Wire.read());
  accelY = (int16_t)(Wire.read() << 8 | Wire.read());
  accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read(); // Temp
  gyroX = (int16_t)(Wire.read() << 8 | Wire.read());
  gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
  gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());
}

void IMU::RollPitchFromAccel(double* roll, double* pitch) {
#ifdef RESTRICT_PITCH
  *roll = atan2((double)accelY, (double)accelZ) * RAD_TO_DEG;
  *pitch = atan((double)-accelX / hypotenuse((double)accelY, (double)accelZ)) * RAD_TO_DEG;
#else
  *roll = atan((double)accelY / hypotenuse((double)accelX, (double)accelZ)) * RAD_TO_DEG;
  *pitch = atan2((double)-accelX, (double)accelZ) * RAD_TO_DEG;
#endif
}

uint32_t IMU::getLastReadTime() { return lastProcessed; }
int16_t IMU::getRawAccelX() { return accelX; }
int16_t IMU::getRawAccelY() { return accelY; }
int16_t IMU::getRawAccelZ() { return accelZ; }
int16_t IMU::getRawGyroX() { return gyroX; }
int16_t IMU::getRawGyroY() { return gyroY; }
int16_t IMU::getRawGyroZ() { return gyroZ; }
double IMU::getRoll() { return kalXAngle; }
double IMU::getPitch() { return kalYAngle; }

// إرجاع القيم بالـ g والـ dps
double IMU::getAccelX_g() { return (double)accelX / 16384.0; }
double IMU::getAccelY_g() { return (double)accelY / 16384.0; }
double IMU::getAccelZ_g() { return (double)accelZ / 16384.0; }
double IMU::getGyroX_dps() { return (double)gyroX / 131.0; }
double IMU::getGyroY_dps() { return (double)gyroY / 131.0; }
double IMU::getGyroZ_dps() { return (double)gyroZ / 131.0; }
