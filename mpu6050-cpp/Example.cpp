//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0

//Example code

#include <MPU6050.h>

MPU6050 device(0x68);

int main() {
	float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values

	sleep(1); //Wait for the MPU6050 to stabilize

/*
	//Calculate the offsets
	std::cout << "Calculating the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	device.getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
	std::cout << "Gyroscope R,P,Y: " << gr << "," << gp << "," << gy << "\nAccelerometer X,Y,Z: " << ax << "," << ay << "," << az << "\n";
*/

	//Read the current yaw angle
	device.calc_yaw = true;
	std::chrono::milliseconds timespan(25);
	
	system("clear");
	while (true)
	{
		//Get the current accelerometer values
		device.getAccelRaw(&ax, &ay, &az);
		std::cout << "\033[1HAccelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << "             \n";

		//Get the current gyroscope values
		device.getGyroRaw(&gr, &gp, &gy);
		std::cout << "Gyroscope Readings: X: " << gr << ", Y: " << gp << ", Z: " << gy << "                  \n";
		std::this_thread::sleep_for(timespan);
	}
	
	

	return 0;
}


