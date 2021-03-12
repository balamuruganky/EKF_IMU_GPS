#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <time.h>
#include <rc/mpu.h>
#include <rc/time.h>

#include "ekfNavINS.h"
#include "SparkFun_Ublox_Arduino_Library.h"

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <tuple>

using namespace std;

#define I2C_BUS 2

static int running = 0;

SFE_UBLOX_GPS gps;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

// file pointer
fstream fout;
inline void open_file(std::string file_name) {
    fout.open(file_name, ios::out | ios::app);
    fout << std::fixed <<"time" << "," << "latitude" << "," << "longitude" << "," 
    								   << "roll" << "," << "pitch" << "," << "yaw" << "," 
    								   << "filtered_latitude" << "," << "filtered_longitude" << ","
    								   << "filtered_roll" << "," << "filtered_pitch" << "," << "filtered_yaw" << std::endl;
}

inline void update_file(long int time, double latitude, double longitude,
						float roll, float pitch, float yaw, 
						double filtered_latitude, double filtered_longitude,
						float filtered_roll, float filtered_pitch, float filtered_yaw) {
	fout << time << "," << std::setprecision(7) << latitude << "," << std::setprecision(7) << longitude 
				 << "," << roll << "," << pitch << "," << yaw
				 << "," << std::setprecision(7) << filtered_latitude << "," << std::setprecision(7) << filtered_longitude 
				 << "," << filtered_roll << "," << filtered_pitch << "," << filtered_yaw << std::endl;
}

inline void close_file() {
	fout.close();
}

int main(int argc, char *argv[])
{
	// MPU9560
	rc_mpu_data_t data; //struct to hold new data
	// Ublox GPS
	Stream serialComm("/dev/ttyACM0");
    gps.begin(serialComm);
    // EKF
    ekfNavINS ekf;

    float ax, ay, az, hx, hy, hz, pitch, roll, yaw;

	// parse arguments
	opterr = 0;

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	// use defaults for now, except also enable magnetometer.
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	conf.enable_magnetometer = 1; // Enable magnetometer
	conf.show_warnings = 1; // Enable warnings

	if(rc_mpu_initialize(&data, conf)){
		fprintf(stderr,"rc_mpu_initialize_failed\n");
		return -1;
	}

	open_file(std::string("./gnss.csv"));

	//now just wait, print_data will run
	while (running) {
		// read sensor data
		if(rc_mpu_read_accel(&data)<0){
			printf("read accel data failed\n");
		}
		if(rc_mpu_read_gyro(&data)<0){
			printf("read gyro data failed\n");
		}
		if(rc_mpu_read_mag(&data)){
			printf("read mag data failed\n");
		}

		// update the filter
		if (gps.getPVT()) {
			ax = data.accel[0];
			ay = -1*data.accel[1];
			az = data.accel[2];
			hx = data.mag[0]; 
			hy = data.mag[1]; 
			hz = data.mag[2];
			std::tie(pitch,roll,yaw) = ekf.getPitchRollYaw(ax, ay, az, hx, hy, hz);
			ekf.ekf_update(time(NULL),gps.getTimeOfWeek(), gps.getNedNorthVel()*1e-3, gps.getNedEastVel()*1e-3, gps.getNedDownVel()*1e-3,
				gps.getLatitude()*1e-7*DEG_TO_RAD, gps.getLongitude()*1e-7*DEG_TO_RAD, (gps.getAltitude()*1e-3),
				data.gyro[0]*DEG_TO_RAD, -1*data.gyro[1]*DEG_TO_RAD, data.gyro[2]*DEG_TO_RAD,
				ax, ay, az, hx, hy, hz);

			update_file(time(NULL), 
						gps.getLatitude()*1e-7, gps.getLongitude()*1e-7,
						roll, pitch, yaw,
						ekf.getLatitude_rad()*RAD_TO_DEG, ekf.getLongitude_rad()*RAD_TO_DEG,
						ekf.getRoll_rad(), ekf.getPitch_rad(), ekf.getHeading_rad());

			printf("------------------------- %ld -------------------------- \n", gps.getTimeOfWeek());
			printf("Latitude  : %2.7f %2.7f\n", gps.getLatitude()*1e-7, ekf.getLatitude_rad()*RAD_TO_DEG);
			printf("Longitute : %2.7f %2.7f\n", gps.getLongitude()*1e-7, ekf.getLongitude_rad()*RAD_TO_DEG);
			printf("Altitude  : %2.3f %2.3f\n", gps.getAltitude()*1e-3, ekf.getAltitude_m());
			printf("Speed (N) : %2.3f %2.3f\n", gps.getNedNorthVel()*1e-3, ekf.getVelNorth_ms());
			printf("Speed (E) : %2.3f %2.3f\n", gps.getNedEastVel()*1e-3, ekf.getVelEast_ms());
			printf("Speed (D) : %2.3f %2.3f\n", gps.getNedDownVel()*1e-3, ekf.getVelDown_ms());
			printf("Roll 	  : %2.3f %2.3f\n", roll, ekf.getRoll_rad());
			printf("Pitch     : %2.3f %2.3f\n", pitch, ekf.getPitch_rad());
			printf("Yaw       : %2.3f %2.3f\n", yaw, ekf.getHeading_rad());
			/*printf("Gyro X    : %f  %f\n", data.gyro[0]*DEG_TO_RAD, ekf.getGyroBiasX_rads());
			printf("Gyro Y    : %f  %f\n", data.gyro[1]*DEG_TO_RAD, ekf.getGyroBiasY_rads());
			printf("Gyro Z    : %f  %f\n", data.gyro[2]*DEG_TO_RAD, ekf.getGyroBiasZ_rads());
			printf("Accel X   : %f  %f\n", data.accel[0], ekf.getAccelBiasX_mss());
			printf("Accel Y   : %f  %f\n", data.accel[1], ekf.getAccelBiasY_mss());
			printf("Accel Z   : %f  %f\n", data.accel[2], ekf.getAccelBiasZ_mss());*/
			printf("-----------------------------------------------------------------\n");
		}
		rc_usleep(100000);
	}
	printf("\n");
	close_file();
	rc_mpu_power_off();
	return 0;
}

