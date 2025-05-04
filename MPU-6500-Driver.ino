#include "MPU6500.h"

void setup() {
  Serial.begin(115200);

  IMU::init();
}

void loop() {
  IMU::read();

  // طباعة القيم بشكل منظم
  Serial.println("------------------------------------------------------------");
  Serial.println("Raw Sensor Data:");
  Serial.print("Accel [X, Y, Z]: ");
  Serial.print(IMU::getRawAccelX()); Serial.print("\t");
  Serial.print(IMU::getRawAccelY()); Serial.print("\t");
  Serial.println(IMU::getRawAccelZ());

  Serial.print("Gyro  [X, Y, Z]: ");
  Serial.print(IMU::getRawGyroX()); Serial.print("\t");
  Serial.print(IMU::getRawGyroY()); Serial.print("\t");
  Serial.println(IMU::getRawGyroZ());

  Serial.println("------------------------------------------------------------");
  Serial.println("Processed Sensor Data:");
  Serial.print("Accel [X, Y, Z] (g): ");
  Serial.print(IMU::getAccelX_g(), 4); Serial.print("\t");
  Serial.print(IMU::getAccelY_g(), 4); Serial.print("\t");
  Serial.println(IMU::getAccelZ_g(), 4);

  Serial.print("Gyro  [X, Y, Z] (°/s): ");
  Serial.print(IMU::getGyroX_dps(), 2); Serial.print("\t");
  Serial.print(IMU::getGyroY_dps(), 2); Serial.print("\t");
  Serial.println(IMU::getGyroZ_dps(), 2);

  Serial.println("------------------------------------------------------------");
  Serial.print("Roll: "); Serial.print(IMU::getRoll(), 2);
  Serial.print("\tPitch: "); Serial.println(IMU::getPitch(), 2);
  Serial.println("------------------------------------------------------------");

  delay(1000);

}

