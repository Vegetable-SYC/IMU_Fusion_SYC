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

  Serial.print("Data_Fusion:");
  Serial.println(imu.Data_Fusion());
}
