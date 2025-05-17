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
  Serial.begin(9600);
  Serial.print("\n");
  Wire.begin();
  // After calibration set the QMC5883L calibration value
  imu.QMC5883L_SetOffsets(-117, -634, 0);
  imu.QMC5883L_SetScales(1.00, 0.91, 0);
  imu.begin(CHOOSE_QMC5883L); // Select QMC5883L
}

void loop() {
  imu.Calculate();
  Serial.println(imu.getHeading());
}