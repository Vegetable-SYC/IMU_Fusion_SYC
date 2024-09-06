/********************************************************

Author: Vegetable_SYC

Address: https://github.com/Vegetable-SYC/IMU_Fusion_SYC

Version: v1.1.6

Please note that the automatic calibration procedure needs
to be executed after the MPU6050 is initialized, and the 
execution order cannot be changed, otherwise an error may
occur!!!!

********************************************************/

#include "IMU_Fusion_SYC.h"

IMU imu(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  imu.begin(CHOOSE_MPU6050);             // Select MPU6050
  // imu.MPU6050_SetGyroOffsets(0, 0, 0);// MPU6050 manually calibrated
  imu.MPU6050_CalcGyroOffsets();         // MPU6050 automatic calibration
}

void loop() {
  imu.Calculate();                       // Calculating Angle
  Serial.println(imu.getAngleZ());
}