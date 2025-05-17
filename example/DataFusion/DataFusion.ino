/********************************************************
  @author: Vegetable_SYC
  @address: https://github.com/Vegetable-SYC/IMU_Fusion_SYC
  @version: v1.2.0
  
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
  imu.QMC5883L_SetOffsets(278.5,573,0);
  imu.QMC5883L_SetScales(1,0.92,0);
  imu.Heading_Offset(360); // Set offest
  imu.begin(CHOOSE_ALL);// IMU initialization
  imu.MPU6050_CalcGyroOffsets();// MPU6050 calibration
}

void loop() {
  // put your main code here, to run repeatedly:
  imu.Calculate();// calculating data
  Serial.println("");

  Serial.print("Data_Fusion:");
  Serial.println(imu.Data_Fusion(0.1));
}
