/**************************************************************************************************

Author: Vegetable_SYC

Address: https://github.com/Vegetable-SYC/IMU_Fusion_SYC

Version: v1.1.6

QMC5883L calibration steps:
Step 1: Start by running the following code directly and viewing serial monitor
step 2: You need to keep turning QMC5883L until the progress bar is full
Step 3: Enter the value obtained in step1 into imu.QMC5883L_SetOffsets and imu.QMC5883L_SetScales
Step 4: Annotation imu.QMC5883L_Calibration();
Step 5: Uncomment the function imu.Calculate and Serial.println(imu.getHeading())
Step 6: Upload the modified program to get the correct Angle value

**************************************************************************************************/
#include "IMU_Fusion_SYC.h"

IMU imu(Wire);

void setup() {
  Serial.begin(9600);
  Serial.print("\n");
  Wire.begin();
  // After calibration set the QMC5883L calibration value
  // imu.QMC5883L_SetOffsets(-117, -634, 0);
  // imu.QMC5883L_SetScales(1.00, 0.91, 0);
  imu.begin(CHOOSE_QMC5883L); // Select QMC5883L
}

void loop() {
  imu.QMC5883L_Calibration();
  // imu.Calculate();
  // Serial.println(imu.getHeading());
}