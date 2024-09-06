/********************************************************

Author: Vegetable_SYC

Address: https://github.com/Vegetable-SYC/IMU_Fusion_SYC

Version: v1.1.6

********************************************************/

#include "IMU_Fusion_SYC.h"

IMU imu(Wire);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);// Set baud rate
  Wire.begin();
  imu.QMC5883L_SetOffsets(-377.5,-199,250);
  imu.QMC5883L_SetScales(1,0.96,0);
  imu.Heading_Offset(360); // Set offest
  imu.begin(CHOOSE_ALL);// IMU initialization
  imu.MPU6050_CalcGyroOffsets();// MPU6050 calibration
}

void loop() {
  // put your main code here, to run repeatedly:
  imu.Calculate();// calculating data
  Serial.println("");
  // Calculated data Output raw accelerometer and gyroscope data
  Serial.print("raw_accx:");
  Serial.print(imu.getraw_accx());
  Serial.print("\t");
  Serial.print("raw_accy:");
  Serial.print(imu.getraw_accy());
  Serial.print("\t");
  Serial.print("raw_accz:");
  Serial.println(imu.getraw_accz());

  Serial.print("raw_gyrox:");
  Serial.print(imu.getraw_gyrox());
  Serial.print("\t");
  Serial.print("raw_gyroy:");
  Serial.print(imu.getraw_gyroy());
  Serial.print("\t");
  Serial.print("raw_gyroz:");
  Serial.println(imu.getraw_gyroz());
  delay(1000);
}
