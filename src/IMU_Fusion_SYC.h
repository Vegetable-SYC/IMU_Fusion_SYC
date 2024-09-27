#ifndef IMU_SYC_H
#define IMU_SYC_H

#include "Arduino.h"
#include "Wire.h"

#define CHOOSE_ALL             0
#define CHOOSE_MPU6050         1
#define CHOOSE_QMC5883L        2

#define MPU6050_ID             0x75// MPU6050 ID
#define MPU6050_ADDR           0x68// MPU6050 Address
#define MPU6050_SMPLRT_DIV     0x19// sample frequency
#define MPU6050_CONFIG         0x1a// Filter frequency
#define MPU6050_GYRO_CONFIG    0x1b// The range of the gyroscope
#define MPU6050_ACCEL_CONFIG   0x1c// The range of the accelerometer
#define MPU6050_WHO_AM_I       0x75// Identity identifier
#define MPU6050_PWR_MGMT_1     0x6b// power management
#define MPU6050_Start_ADDR     0x3b// Data origin address

#define QMC5883L_ADDR          0x0D// QMC5883L address
#define QMC5883L_WORK_MODE     0x0B// QMC5883L Working mode address
#define QMC5883L_CONTROL_ADDR  0x09// QMC5883L control address
#define QMC5883L_Start_ADDR    0x00// QMC5883L start address
class IMU{
  public:

  IMU(TwoWire &i);

  void begin(uint8_t choose = CHOOSE_ALL);

  void MPU6050_SetGyroOffsets(float x, float y, float z);
  void MPU6050_CalcGyroOffsets();
  void calcGyroOffsets();
  void Calculate();

  float getQMCRawx();
  float getQMCRawy();
  float getQMCRawz();

  int16_t getraw_accx();
  int16_t getraw_accy();
  int16_t getraw_accz();

  int16_t getraw_gyrox();
  int16_t getraw_gyroy();
  int16_t getraw_gyroz();

  float getaccx();
  float getaccy();
  float getaccz();

  float getgyrox();
  float getgyroy();
  float getgyroz();

  float getAcc_AngleX();
  float getAcc_AngleY();

  float getGyro_AngleX();
  float getGyro_AngleY();
  float getGyro_AngleZ();

  float getAngleX();
  float getAngleY();
  float getAngleZ();

  float getAccMagnitude();

  void QMC5883L_SetOffsets(float x, float y, float z);
  void QMC5883L_SetScales(float x, float y, float z);
  void QMC5883L_Calibration();
  int Data_Fusion(float alpha);
  void Error_compensation();
  void ComplementaryFilter(int acc_z, float gyro, float dt, int acc_heading);
  void Heading_Offset(int offest);
  int getHeading();

  byte I2C_Read(byte reg,byte addr);
  void I2C_Write(byte reg, byte data, byte addr);
	
  private:

  TwoWire *wire;

  bool MPU6050_state = false, QMC5883L_state = false;

  int16_t raw_accx, raw_accy, raw_accz, raw_gyrox, raw_gyroy, raw_gyroz, rawTemp;

  float gx_offset, gy_offset, gz_offset;

  float accx, accy, accz, gyrox, gyroy, gyroz;

  float Gyro_AngleX, Gyro_AngleY, Gyro_AngleZ, Acc_AngleX, Acc_AngleY, Acc_AngleZ;

  float AngleX, AngleY, AngleZ;

  int Angle_Round, Angle_Absolute, heading_offset;

  float QMC_RawX, QMC_RawY, QMC_RawZ, QMC_X, QMC_Y, QMC_Z, heading; 
  float x_offset, y_offset, z_offset, x_scale = 1, y_scale = 1, z_scale = 1;
  float accMagnitude;

  uint32_t previousMillis;

  unsigned long previoustime = 0; // Time of the last update
  const long val = 600;           // Interval of each progress bar section (600 ms)
  const int barLength = 50;       // Progress bar length
  int currentProgress = 0;        // Current progress
  float qmc_xmax, qmc_xmin, qmc_ymax, qmc_ymin, qmc_zmax, qmc_zmin;
  float QMCx_offset, QMCy_offset, QMCz_offset;
  float QMCx_scale, QMCy_scale, QMCz_scale;

  int Angle_Fusion, new_angle;

  float a,b = 0.1;  
  float interval;
  long preInterval;
};

#endif
