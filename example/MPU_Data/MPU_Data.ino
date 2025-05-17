/********************************************************
  @author: Vegetable_SYC
  @address: https://github.com/Vegetable-SYC/IMU_Fusion_SYC
  @version: v1.2.1
  
  @note:
   Please note that the automatic calibration procedure needs
   to be executed after the MPU6050 is initialized, and the 
   execution order cannot be changed, otherwise an error may
   occur!!!!
   
   If you find that some boards are not working, 
   you can contact me on GitHub or send me an email at 
   1318270340@qq.com.
 *******************************************************/

#include "IMU_Fusion_SYC.h"

IMU imu(Wire);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);// Set baud rate
  Wire.begin();
  imu.begin(CHOOSE_MPU6050);// IMU initialization
  imu.MPU6050_CalcGyroOffsets();// MPU6050 calibration
}

void loop() {
  // put your main code here, to run repeatedly:
  imu.Calculate();// calculating data
  Serial.println("\n****************************************************\n");
  // Calculated data Output raw accelerometer and gyroscope data
  Serial.print("accx:");
  Serial.print(imu.getaccx());
  Serial.print("\t");
  Serial.print("accy:");
  Serial.print(imu.getaccy());
  Serial.print("\t");
  Serial.print("accz:");
  Serial.println(imu.getaccz());

  Serial.print("gyrox:");
  Serial.print(imu.getgyrox());
  Serial.print("\t");
  Serial.print("gyroy:");
  Serial.print(imu.getgyroy());
  Serial.print("\t");
  Serial.print("gyroz:");
  Serial.println(imu.getgyroz());
  Serial.println("\n****************************************************\n");
  delay(1000);
}