# Extended Kalman Filter (GPS, Velocity and IMU fusion)

## Goal
The goal of this algorithm is to enhance the accuracy of GPS reading based on IMU reading. No RTK supported GPS modules accuracy should be equal to greater than 2.5 meters. Extended Kalman Filter algorithm shall fuse the GPS reading (Lat, Lng, Alt) and Velocities (Vn, Ve, Vd) with 9 axis IMU to improve the accuracy of the GPS.

## Dependencies
* CMake
* Boost
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [librobotcontrol](https://github.com/beagleboard/librobotcontrol) (Optional to compile and execute test executable in Beaglebone Blue)

## How to
### Compile
* git clone https://github.com/balamuruganky/EKF_IMU_GPS
* cd EKF_IMU_GPS
* git submodule update --init --recursive
* mkdir build
* cd build
* cmake ..
* make

### Execute
Beaglebone Blue board is used as test platform. There is an inboard MPU9250 IMU and related library to calibrate the IMU. Please go through librobotcontrol [documentation](http://beagleboard.org/librobotcontrol/) for more information.
If you are having Beaglebone Blue board, then connect Ublox GPS through USB to test the EKF filter as mentioned below,

* Compile the source code as mentioned in the above section
* ../bin/ekf_test

## EKF 15 States

### Inputs (Make sure IMU fixed in NED frame)
1) Latitude, units are rad
2) Longitude, units are rad
3) Altitude, units are m
4) Velocity (North), units are m/s
5) Velocity (East), units are m/s
6) Velocity (Down), units are m/s
7) Accelarometer X, units are m/s/s
8) Accelarometer Y, units are m/s/s
9) Accelarometer Z, units are m/s/s
10) Gyro X, units are rad/s
11) Gyro Y, units are rad/s
12) Gyro Z, units are rad/s
13) Magnetometer X, units need to be consistant across all magnetometer measurements used (eg. mT)
14) Magnetometer Y, units need to be consistant across all magnetometer measurements used (eg. mT)
15) Magnetometer Z, units need to be consistant across all magnetometer measurements used (eg. mT)

### Outputs
1) double getLatitude_rad()
2) double getLongitude_rad()
3) double getAltitude_m()
4) double getVelNorth_ms()
5) double getVelEast_ms()
6) double getVelDown_ms()
7) float  getRoll_rad()
8) float  getPitch_rad()
9) float  getHeading_rad()
10) float  getGroundTrack_rad()

### Bias Outputs
1) float getGyroBiasX_rads()
2) float getGyroBiasY_rads()
3) float getGyroBiasZ_rads()
4) float getAccelBiasX_mss()
5) float getAccelBiasY_mss()
6) float getAccelBiasZ_mss()

## Result
The ekf_test executable produce gnss.txt file that contains the raw and filtered GPS coordinates. Python utils developed to visualize the EKF filter performance. Sample result shown below.

![alt text][ekf_result]

[ekf_result]: https://github.com/balamuruganky/extended_kalman_filter/blob/master/python_utils/test_data/raw_vs_ekf.png "GPS Raw Vs Filtered"

## Credits
* https://github.com/UASLab/OpenFlight/blob/master/FlightCode/navigation/EKF_15state_quat.c
* https://github.com/FlyTheThings/uNavINS
