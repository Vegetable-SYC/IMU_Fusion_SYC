# IMU_Fusion_SYC   v1.1.5

The data of MPU6050 and QMC5883L can be read, and the data fusion of both can be realized, In github, I also provide a QMC5883L calibration value calculation [script](https://github.com/Vegetable-SYC/IMU_Fusion_SYC), which also contains the calibration method,Currently, only Chinese and English are supported.

## **How** to use?

you can see [example sketch](https://github.com/Vegetable-SYC/IMU_Fusion_SYC), It includes how to read the data and how to get the merged data.



## Attention

Note that it is best to execute the method other than the gyroscope automatic calibration before running the begin method to avoid data calculation errors



### IMU_INIT

In this library, the data of MPU6050 and QMC5883L can be read separately or at the same time. However, only the same I2C interface can be used when the two sensors are read at the same time. If the data of only one sensor needs to be read, you can set it by using the begin() method

#### example

You can change the parameters in the begin method to determine the sensor to be initialized. The parameters are: initialize both sensors, initialize only QMC5883L, and initialize only MPU6050. Note that you only need to call the begin method once in your project.

```c
#include "IMU_Fusion_SYC.h"

IMU imu(Wire);

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  imu.begin(CHOOSE_ALL);		// Both sensors are initialized
  //imu.begin(CHOOSE_MPU6050);	// Only MPU6050 is initialized
  //imu.begin(CHOOSE_QMC5883L);	// Only QMC5883L is initialized
}
```

If both sensors are successfully initialized, the following output is displayed on the serial monitor

```c
***********************************************************

QMC_Init!!!

MPU_Init!!!

***********************************************************
```

If the sensor fails to be initialized, the following output is displayed on the serial port monitor

```c
*****************************************************************

QMC_Init false!!! Please check whether the connection is correct

MPU_Init false!!! Please check whether the connection is correct

*****************************************************************
```

Note that if only one sensor is initialized individually, only information about the corresponding sensor will be output ! ! !



### QMC5883L calibration

QMC5883L should be calibrated before use, otherwise correct readings cannot be obtained. The obtained Offest and Scale should be recorded and filled in the QMC5883L_SetOffsets method and the QMC5883L_SetScales method

#### example

```c
#include "IMU_Fusion_SYC.h"

IMU imu(Wire);

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  imu.begin(CHOOSE_QMC5883L);			// Only QMC5883L is initialized
  imu.QMC5883L_SetOffsets(***,***,***);
  imu.QMC5883L_SetScales(*,*,*);
}
```

### QMC5883L headingoffest

The different placement angles of QMC5883L relative to the magnetic field will also affect the final output Angle, and the corresponding deviation can be set in the Heading_Offset method

#### example

```c
#include "IMU_Fusion_SYC.h"

IMU imu(Wire);

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  imu.Heading_Offset(360); 			// Set Heading offest
  imu.begin(CHOOSE_QMC5883L);		// Only QMC5883L is initialized
}
```



### MPU6050 calibration

The gyroscope calibration of the MPU6050 is automatic. After calling the MPU6050_CalcGyroOffsets method, the MPU6050 only needs to be placed on the horizontal surface. During the calibration, the MPU6050 should not be moved

#### example

```c
#include "IMU_Fusion_SYC.h"

IMU imu(Wire);

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  imu.begin(CHOOSE_MPU6050);				// Only MPU6050 is initialized
  imu.MPU6050_CalcGyroOffsets();			// MPU6050 calibration
}
```

The serial port monitor will output the following information during calibration

```c
***********************************************************
MPU6050 is being calibrated
Uneven placement can cause data errors

Calibration complete!
X_offest : 6.14
Y_offest : 1.02
Z_offest : 2.11
***********************************************************
```



### MPU6050 Sets the calibration value

If you know the deviation of the MPU6050 gyroscope, you can choose to manually set the deviation, so that the device initialization time will be greatly reduced.

#### example

```c
#include "IMU_Fusion_SYC.h"

IMU imu(Wire);

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  imu.begin(CHOOSE_MPU6050);			  // Only MPU6050 is initialized
  imu.MPU6050_SetGyroOffsets(***,***,***);// MPU6050 calibration value
}
```



### Data_Fusion

A simple data fusion scheme is provided in the library, which can effectively fuse the data of the two sensors, and the combination of the two can effectively suppress the zero drift of the MPU6050 and enhance the accuracy of the Angle of motion.

#### example

```c
#include "IMU_Fusion_SYC.h"

IMU imu(Wire);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);// Set baud rate
  Wire.begin();
  imu.QMC5883L_SetOffsets(***,***,***);
  imu.QMC5883L_SetScales(*,*,*);
  imu.Heading_Offset(360); 					// Set offest
  imu.begin(CHOOSE_ALL);					// IMU initialization
  imu.MPU6050_CalcGyroOffsets();// MPU6050 calibration
}

void loop() {
  // put your main code here, to run repeatedly:
  imu.Calculate();							// calculating data
  Serial.println("");

  Serial.print("Data_Fusion:");
  Serial.println(imu.Data_Fusion());
}
```



## Licence

MIT

## Author

Vegetable-SYC