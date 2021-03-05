# uNavINS
uNav Inertial Navigation System 15 State EKF Arduino Library.

***Developing with the [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) or [LC](https://www.pjrc.com/store/teensylc.html)? Consider buying our [Teensy Motion Backpack](http://bolderflight.com/products/teensy/motion/), which integrates an MPU-9250 and BME-280 into a stackable add-on module, along with our [Teensy GNSS Backpack](http://bolderflight.com/products/teensy/gnss/), which integrates the uBlox SAM-M8Q GNSS module and provides UART, I2C, and PPS interfaces. All of our Backpacks are thoughtfully designed to integrate perfectly with the Teensy. Check out all of our wonderfully small and powerful [Teensy Backpacks](http://bolderflight.com/products/teensy/)***

# Description
The uNav Inertial Navigation System (INS) is a 15 state Extended Kalman Filter (EKF) to estimate the following from IMU and GPS data:
* Attitude
* Latitude, longitude, and altitude (LLA)
* North, east, down (NED) inertial velocity
* Ground track 

The 15 states comprise the inertial position, inertial velocity, a quaternion, accelerometer biases, and gyro biases. This algorithm was developed by Adhika Lie at the [University of Minnesota UAS Research Labs](http://www.uav.aem.umn.edu), where it has been used since 2006 as a baseline navigation algorithm to gauge the performance of other algorithms in simulation studies and flight tests. uNav INS provides excellent estimates of attitude, inertial position, and inertial velocity once it has converged on a solution. 

XXX - how to tell that solution has converged? How to warm up filter?

# Usage

## Installation
This library requires [Eigen](https://github.com/bolderflight/Eigen) to compile. First download or clone [Eigen](https://github.com/bolderflight/Eigen) into your Arduino/libraries folder, then download or clone this library into your Arduino/libraries folder. Additionally, this library requires IMU measurements and GPS measurements. For the included examples, an [MPU-9250 IMU](https://github.com/bolderflight/MPU9250) is used with a [uBlox GPS](https://github.com/bolderflight/UBLOX), and their libraries will need to be installed as well. 

XXX - Finally, because this library is using accelerometers and magnetometers as a measurement update, the IMU used should be well calibrated.

## Function Description

### Object Declaration
This library uses the default constructor. The following is an example of declaring a uNavINS object called *Filter*.

```C++
uNavINS Filter;
```

### Data Collection Functions

**void update(unsigned long TOW,double vn,double ve,double vd,double lat,double lon,double alt,float p,float q,float r,float ax,float ay,float az,float hx,float hy, float hz)** updates the filter with new IMU and GPS measurements, time updates propogate the state and measurement updates are made; the attitude and inertial position and velocity estimates of the vehicle are updated. Inputs are:

* unsigned long TOW: GPS time of week. This is used to trigger a measurement update when the GPS data is new (i.e. the TOW has changed).
* double vn: GPS velocity in the North direction, units are m/s.
* double ve: GPS velocity in the East direction, units are m/s.
* double vd: GPS velocity in the down direction, units are m/s.
* double lat: GPS measured latitude, units are rad.
* double lon: GPS measured longitude, units are rad.
* double alt: GPS measured altitude, units are m.
* float p: gyro measurement in the x direction, units are rad/s.
* float q: gyro measurement in the y direction, units are rad/s.
* float r: gyro measurement in the z direction, units are rad/s.
* float ax: accelerometer measurement in the x direction, units are m/s/s.
* float ay: accelerometer measurement in the y direction, units are m/s/s.
* float az: accelerometer measurement in the z direction, units are m/s/s.
* float hx: magnetometer measurement in the x direction, units need to be consistant across all magnetometer measurements used.
* float hy: magnetometer measurement in the y direction, units need to be consistant across all magnetometer measurements used.
* float hz: magnetometer measurement in the z direction, units need to be consistant across all magnetometer measurements used.

Please note that IMU measurements need to be given in the [defined axis system](#axis-system).

```C++
// read the sensor
Imu.readSensor();
// update the filter
Filter.update(uBloxData.iTOW,uBloxData.velN,uBloxData.velE,uBloxData.velD,uBloxData.lat*PI/180.0f,uBloxData.lon*PI/180.0f,uBloxData.hMSL,Imu.getGyroY_rads(),-1*Imu.getGyroX_rads(),Imu.getGyroZ_rads(),Imu.getAccelY_mss(),-1*Imu.getAccelX_mss(),Imu.getAccelZ_mss(),Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT());
```

**float getRoll_rad()** returns the roll angle in units of rad.

```C++
float roll;
roll = Filter.getRoll_rad();
```

**float getPitch_rad()** returns the pitch angle in units of rad.

```C++
float pitch;
pitch = Filter.getPitch_rad();
```

**float getYaw_rad()** returns the yaw angle in units of rad.

```C++
float yaw;
yaw = Filter.getYaw_rad();
```

**float getHeading_rad()** returns the heading angle in units of rad.

```C++
float heading;
heading = Filter.getHeading_rad();
```

**double getLatitude_rad** returns the INS estimated latitude in units of rad.

```C++
double latitude;
latitude = Filter.getLatitude_rad();
```

**double getLongitude_rad** returns the INS estimated longitude in units of rad.

```C++
double longitude;
longitude = Filter.getLongitude_rad();
```

**double getAltitude_m** returns the INS estimated altitude in units of m.

```C++
double alt;
alt = Filter.getAltitude_m();
```

**double getVelNorth_ms** returns the INS estimated inertial velocity in the North direction in units of m/s.

```C++
double vn;
vn = Filter.getVelNorth_ms();
```

**double getVelEast_ms** returns the INS estimated inertial velocity in the East direction in units of m/s.

```C++
double ve;
ve = Filter.getVelEast_ms();
```

**double getVelDown_ms** returns the INS estimated inertial velocity in the down direction in units of m/s.

```C++
double vd;
vd = Filter.getVelDown_ms();
```

**float getGroundTrack_rad** returns the INS estimated inertial ground track in units of rad.

```C++
float track;
track = Filter.getGroundTrack_rad();
```

**float getGyroBiasX_rads** returns the current gyro bias in the x direction in units of rad/s.

```C++
float gxb;
gxb = Filter.getGyroBiasX_rads();
```

**float getGyroBiasY_rads** returns the current gyro bias in the y direction in units of rad/s.

```C++
float gyb;
gyb = Filter.getGyroBiasY_rads();
```

**float getGyroBiasZ_rads** returns the current gyro bias in the z direction in units of rad/s.

```C++
float gzb;
gzb = Filter.getGyroBiasZ_rads();
```

**float getAccelBiasX_mss** returns the current accelerometer bias in the x direction in units of m/s/s.

```C++
float axb;
axb = Filter.getAccelBiasX_mss();
```

**float getAccelBiasY_mss** returns the current accelerometer bias in the Y direction in units of m/s/s.

```C++
float ayb;
ayb = Filter.getAccelBiasY_mss();
```

**float getAccelBiasZ_mss** returns the current accelerometer bias in the Z direction in units of m/s/s.

```C++
float azb;
azb = Filter.getAccelBiasZ_mss();
```

## <a name="axis-system"></a>Axis System
This library expects IMU data to be input in a defined axis system, which is shown below. It is a right handed coordinate system with x-axis pointed forward, the y-axis to the right, and the z-axis positive down, common in aircraft dynamics. Pitch is defined as a rotation angle around the y-axis with level as zero and roll is defined as a rotation angle around the x-axis with level as zero. Yaw is defined as a rotation angle around the z-axis with zero defined as the starting orientation. Heading is defined as a rotation angle around the z-axis with zero defined as magnetic north.

<img src="https://github.com/bolderflight/MPU9250/blob/master/extras/MPU-9250-AXIS.png" alt="Common Axis System" width="250">

## Example List
* **uNavINS-with-MPU9250**: demonstrates using this filter with an MPU-9250 IMU. *CalibrateMPU9250.ino* is used to calibrate the MPU-9250 IMU and store the calibration coefficients in EEPROM. *uNavINS_MPU9250.ino* uses the MPU-9250 IMU and uBlox GPS as measurement inputs to the uNav INS filter, which is run at a rate of 100 Hz. 

# <a name="references">References 

1. FILL
