/**********************************************************************
  Filename    : IMU_SYC
  Description : The data of MPU6050 and QMC5883L can be read, and the 
                data fusion of both can be realized
  Versions    : v1.0.0
  Auther      : Vegtable SYC
  Modification: 2024/08/24
**********************************************************************/

#include "IMU_Fusion_SYC.h"
#include "Arduino.h"

IMU::IMU(TwoWire &i){//Get wire
  wire = &i;
}

void IMU::begin(uint8_t choose){
  delay(2000);
  Serial.println("***********************************************************");
  // QMC58883L initialized
  if(choose == CHOOSE_QMC5883L || choose == CHOOSE_ALL)
  {
    
    Serial.println("");
    Wire.beginTransmission(QMC5883L_ADDR);
    bool error = Wire.endTransmission();
    if(error != 0)
    {
      Serial.println("QMC_Init false!!! \t Please check whether the connection is correct");
    }else{
      Serial.println("QMC_Init!!!");
      QMC5883L_state = true;
      I2C_Write(QMC5883L_WORK_MODE, 0x01, QMC5883L_ADDR);
      I2C_Write(QMC5883L_CONTROL_ADDR, 0x01 | 0x0C | 0x10 | 0x00, QMC5883L_ADDR);
    }
  }
  // MPU6050 initialized
  if(choose == CHOOSE_MPU6050 || choose == CHOOSE_ALL)
  {
    Serial.println("");
    if(I2C_Read(MPU6050_ID, MPU6050_ADDR) != MPU6050_ADDR)
    {
      Serial.println("MPU_Init false!!! \t Please check whether the connection is correct");
    }else{
      Serial.println("MPU_Init!!!");
      MPU6050_state = true;
      I2C_Write(MPU6050_SMPLRT_DIV, 0x00, MPU6050_ADDR);
      I2C_Write(MPU6050_CONFIG, 0x00, MPU6050_ADDR);
      I2C_Write(MPU6050_GYRO_CONFIG, 0x08, MPU6050_ADDR);
      I2C_Write(MPU6050_ACCEL_CONFIG, 0x00, MPU6050_ADDR);
      I2C_Write(MPU6050_PWR_MGMT_1, 0x01, MPU6050_ADDR);
    }
  }
  Serial.println("");
  Serial.println("***********************************************************");
  delay(1500);
  IMU::Calculate();
  Angle_Absolute = heading;
}

void IMU::I2C_Write(byte reg, byte data, byte addr){
  wire->beginTransmission(addr);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();
}

byte IMU::I2C_Read(byte reg,byte addr) {
  wire->beginTransmission(addr);
  wire->write(reg);
  wire->endTransmission(false);
  wire->requestFrom(addr, 1);
  byte data =  wire->read();
  return data;
}

void IMU::MPU6050_SetGyroOffsets(float x, float y, float z){
  gx_offset = x;
  gy_offset = y;
  gz_offset = z;
}

void IMU::MPU6050_CalcGyroOffsets(byte addr){
  int16_t gx = 0, gy = 0, gz = 0;
  float add_x,add_y,add_z;
  delay(2000);
  Serial.println("***********************************************************");
  Serial.println("MPU6050 is being calibrated");
  Serial.println("Uneven placement can cause data errors");

  for(int i = 0; i < 3000; i++){
    if(i % 500 == 0){
      Serial.print(".");
    }
    wire->beginTransmission(addr);
    wire->write(0x43);// Read the gyroscope
    wire->endTransmission(false);
    wire->requestFrom((int)addr, 6);// Read 6 bits consecutively

    gx = wire->read() << 8 | wire->read();
    gy = wire->read() << 8 | wire->read();
    gz = wire->read() << 8 | wire->read();

    add_x += ((float)gx) / 65.5;
    add_y += ((float)gy) / 65.5;
    add_z += ((float)gz) / 65.5;
  }
  gx_offset = add_x / 3000.0;
  gy_offset = add_y / 3000.0;
  gz_offset = add_z / 3000.0;

  Serial.println();
  Serial.println("Calibration complete!");
  Serial.print("X_offest : ");Serial.println(gx_offset);
  Serial.print("Y_offest : ");Serial.println(gy_offset);
  Serial.print("Z_offest : ");Serial.println(gz_offset);
  Serial.print("***********************************************************");
  delay(1000);
}

void IMU::Heading_Offset(int offest)
{
  heading_offset = offest;
}

void IMU::Calculate() {
  if(QMC5883L_state == true)
  {
    // QMC5883L Read
    wire->beginTransmission(QMC5883L_ADDR);
    wire->write(QMC5883L_Start_ADDR);
    wire->endTransmission(false); // End the transfer, but keep the connection
    wire->requestFrom((int)QMC5883L_ADDR, 6);
    QMC_RawX = (int16_t)(Wire.read() | Wire.read() << 8);
    QMC_RawY = (int16_t)(Wire.read() | Wire.read() << 8);
    QMC_RawZ = (int16_t)(Wire.read() | Wire.read() << 8);

    QMC_X = (QMC_RawX - x_offset) * x_scale;
    QMC_Y = (QMC_RawY - y_offset) * y_scale;
    QMC_Z = (QMC_RawZ - z_offset) * z_scale;

    float heading_temp = atan2( QMC_Y, QMC_X ) * 180.0 / PI;
    heading = (int)heading_temp % 360;
    if(heading < 0)
      heading += heading_offset;
  }

  // MPU6050 Read
  if(MPU6050_state == true)
  {
      wire->beginTransmission(MPU6050_ADDR); // Start the communication with the MPU6050
      wire->write(MPU6050_Start_ADDR); // Set the register address to 0x3b (high byte of the accelerometer X axis)
      wire->endTransmission(false); 
      wire->requestFrom((int)MPU6050_ADDR, 14); // Request to read 14 bytes of data from the MPU6050


      // Read the accelerometer data
      raw_accx = wire->read() << 8 | wire->read(); // Read the X-axis acceleration
      raw_accy = wire->read() << 8 | wire->read(); // Read the Y-axis acceleration
      raw_accz = wire->read() << 8 | wire->read(); // Read the Z-axis acceleration
      // Read the temperature data
      rawTemp  = wire->read() << 8 | wire->read();
      // Read gyroscope data
      raw_gyrox = wire->read() << 8 | wire->read(); // Read the X-axis gyroscope
      raw_gyroy = wire->read() << 8 | wire->read(); // Read the Y-axis gyroscope
      raw_gyroz = wire->read() << 8 | wire->read(); // Read the Z-axis gyroscope

      // Converts the raw acceleration data to g units
      accx = ((float)raw_accx) / 16384.0; // Convert the X-axis acceleration
      accy = ((float)raw_accy) / 16384.0; // Convert the Y-axis acceleration
      accz = ((float)raw_accz) / 16384.0; // Convert the Z-axis acceleration

      // Converts raw gyroscope data into units of degrees per second
      gyrox = ((float)raw_gyrox) / 65.5; // Convert the X-axis gyroscope
      gyroy = ((float)raw_gyroy) / 65.5; // Convert the Y-axis gyroscope
      gyroz = ((float)raw_gyroz) / 65.5; // Convert the Z-axis gyroscope

      // Subtract offset from gyroscope data
      gyrox -= gx_offset;
      gyroy -= gy_offset;
      gyroz -= gz_offset;

      // The calculation is based on the Angle of the accelerometer
      Acc_AngleX = atan2(accy, accz + abs(accx)) * 360 / 2.0 / PI; // Calculate the X-axis Angle
      Acc_AngleY = atan2(accx, accz + abs(accy)) * 360 / -2.0 / PI; // Calculate the Y-axis Angle

      // Computation interval
      interval = (millis() - preInterval) * 0.001; // Calculate the time since the last read (in seconds)

      // Update the gyroscope Angle
      Gyro_AngleX += gyrox * interval; // Update the X-axis Angle
      Gyro_AngleY += gyroy * interval; // Update the Y-axis Angle
      Gyro_AngleZ += gyroz * interval; // Update the Z-axis Angle

      // Accelerometer and gyroscope data are combined to update the total Angle
      AngleX = (AngleX + gyrox * interval) + Acc_AngleX; // Updated total X-axis Angle
      AngleY = (AngleY + gyroy * interval) + Acc_AngleY; // Updated total Y-axis Angle
      AngleZ = Gyro_AngleZ;                              // Z-axis Angle directly uses the gyroscope Angle

      preInterval = millis(); // Update the time of the last read
  }
}
int IMU::Data_Fusion() {
  // Calculate the accelerometer value
  float accMagnitude = sqrt(accx * accx + accy * accy + accz * accz);

  // Detect the static state
  bool isStationary = (abs(accMagnitude - 0.98) < 0.03);

  // Dynamically adjust the trust parameters of the complementary filter
  if (isStationary) {
    a = 0;
    b = 0.2;
  }else{
    a = 0.96;
    b = 0.1;
  }

  if(AngleZ < -360)
  {
    Angle_Round = abs(AngleZ) / 360;
    AngleZ += Angle_Round*360;
  }else if(AngleZ > 360){
    Angle_Round = AngleZ / 360;
    AngleZ -= Angle_Round*360;
  }
  if(AngleZ <= 0)
  {
    AngleZ = map(AngleZ,0,-360,0,360);
  }else{
    AngleZ = map(AngleZ,0,360,360,0);
  }
  AngleZ = AngleZ + Angle_Absolute;
  if(AngleZ > 360)
    AngleZ -= 360;
  // Complementary filtering
  IMU::ComplementaryFilter(AngleZ, gyroz, 0.01, heading);
  Angle_Fusion = (1-b) * AngleZ + b * heading;
  // Error repair
  IMU::Error_compensation();

  return Angle_Fusion;
}

void IMU::Error_compensation()
{
  if(previousMillis == 0)
    previousMillis = millis();
  if(millis() - previousMillis > 3000)
  {
    Angle_Absolute += (heading - AngleZ)/6;
    previousMillis = 0;
  }
}

void IMU::ComplementaryFilter(int acc_z, float gyro, float dt, int acc_heading)
{
  AngleZ = a * (acc_z + gyro * dt) + (1 - a) * (heading);
}
// Set offset
void IMU::QMC5883L_SetOffsets(float x, float y, float z) {
	x_offset = x;
	y_offset = y;
	z_offset = z;
}
// Set Scales
void IMU::QMC5883L_SetScales(float x, float y, float z) {
	x_scale = x;
	y_scale = y;
	z_scale = z;
}

int16_t IMU::getraw_accx(){return raw_accx;}
int16_t IMU::getraw_accy(){return raw_accy;}
int16_t IMU::getraw_accz(){return raw_accz;}

int16_t IMU::getraw_gyrox(){return raw_gyrox;}
int16_t IMU::getraw_gyroy(){return raw_gyroy;}
int16_t IMU::getraw_gyroz(){return raw_gyroz;}

float IMU::getaccx(){return accx;}
float IMU::getaccy(){return accx;}
float IMU::getaccz(){return accx;}

float IMU::getgyrox(){return gyrox;}
float IMU::getgyroy(){return gyroy;}
float IMU::getgyroz(){return gyroz;}

float IMU::getAcc_AngleX(){return Acc_AngleX;}
float IMU::getAcc_AngleY(){return Acc_AngleY;}

float IMU::getGyro_AngleX(){return Gyro_AngleX;}
float IMU::getGyro_AngleY(){return Gyro_AngleY;}
float IMU::getGyro_AngleZ(){return Gyro_AngleZ;}

float IMU::getAngleX(){return AngleX;}
float IMU::getAngleY(){return AngleY;}
float IMU::getAngleZ(){return AngleZ;}

int IMU::getHeading(){return heading;}