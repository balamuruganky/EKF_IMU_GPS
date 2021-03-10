# extended_kalman_filter (15 states)

### Goal
The goal of this algorithm is to enhance the accuracy of GPS reading based on IMU reading. No RTK supported GPS modules accuacy should be equal to greater than 2.5 meters. Extended Kalman Filter algorithm shall fuse the GPS reading with 9 axis IMU to improve the accuracy of the GPS.

### Dependencies
* CMake
* Boost
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
* [librobotcontrol](https://github.com/beagleboard/librobotcontrol) (Optional to compile and execute test executable in Beaglebone Blue)

### How to
## Compile
* git clone https://github.com/balamuruganky/extended_kalman_filter
* cd extended_kalman_filter
* git submodule update --init --recursive
* mkdir build
* cd build
* cmake ..
* make

## Execute
Beaglebone Blue board is used as test platform. There is an inboard MPU9250 IMU and related library to calibrate the IMU. Please go through librobotcontrol [documentation](http://beagleboard.org/librobotcontrol/) for more information.
If you are having Beaglebone Blue board, then connect Ublox GPS through USB to test the EKF filter as mentioned below,

* Compile the source code as mentioned in the above section
* ../bin/ekf_test

### EKF 15 States
## Inputs
1) Latitude
2) Longitude
3) Altitude
4) Velocity (North)
5) Velocity (East)
6) Velocity (Down)
7) Accelarometer X
8) Accelarometer Y
9) Accelarometer Z
10) Gyro X
11) Gyro Y
12) Gyro Z
13) Magnetometer X
14) Magnetometer Y
15) Magnetometer Z

## Outputs
1) double getLatitude_rad()
2) double getLongitude_rad()
3) double getAltitude_m()
4) double getVelNorth_ms()
5) double getVelEast_ms()
6) double getVelDown_ms()
7) float  getRoll_rad()
8) float  getPitch_rad()
9) float  getHeading_rad()
10)float  getGroundTrack_rad()

## Bias Outputs
1) float getGyroBiasX_rads()
2) float getGyroBiasY_rads()
3) float getGyroBiasZ_rads()
4) float getAccelBiasX_mss()
5) float getAccelBiasY_mss()
6) float getAccelBiasZ_mss()

### Result
The ekf_test executable produce gnss.txt file that contains the raw and filtered GPS coordinates. Python utils developed to visualize the EKF filter performance. Sample result shown below.

![alt text][ekf_result]

[ekf_result]: https://github.com/balamuruganky/extended_kalman_filter/blob/master/python_utils/test_data/raw_vs_ekf.png "GPS Raw Vs Filtered"

### Credits
* https://github.com/UASLab/OpenFlight/blob/master/FlightCode/navigation/EKF_15state_quat.c
* https://github.com/FlyTheThings/uNavINS
