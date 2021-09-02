#include <Arduino.h>
#include "mpu9250_mark.h"
#include "SPI_ArduinoDue.h"

void printCalibrationOfGyro();
void printCalibrationOfAccelerometer();
void printCalibrationOfMagnetometer();

mpu9250_mark imu(spi_cs_imu_due, write_SPI0_blocking, delay, delayMicroseconds);

void setup() {
	
	Serial.begin(115200);
	
	setup_SPI0_Master(0);
	SPI0_Clock_Rate_khz(400);
	setup_PIOA25_as_SPI0_MISO();
	setup_PIOA26_as_SPI0_MOSI();
	setup_PIOA27_as_SPI0_SCK();
	setup_PIOA28_as_cs_digital();
	
	do{
		int init_val;
		while ((init_val = imu.init_brian()) <= 0){
			Serial.println("failed to restart mpu");
			Serial.println(init_val);
		}
		Serial.println("The return status of brians init is:...");
		Serial.println(init_val);
		imu.init();
		imu.init_mag();
	} while(imu.device_id_mag() != 72 || imu.whoAmI() != 115 );
	imu.gyro_2000dps_range();
	imu.update_accel_bias(0, 0, 0);
	imu.update_accel_mult_bias(1, 1, 1);
	imu.update_gyro_bias(6, 0, 1);
	imu.update_mag_bias(379, -158, -47);
	imu.set_xyz_mag_adj_factor(1, 1.013865, 1.071429);
	
	// functions to help with calibration - danger: infinite loop
	//printCalibrationOfGyro();
	//printCalibrationOfAccelerometer();
	//printCalibrationOfMagnetometer();
}

void loop(){
	
	int16_t gyro_angles[3];
	float gyro_angles_dps[3];
	int16_t accel_values[3];
	float accel_mps2[3];
	float mag_values[3];
	
	
	imu.readAll();
	imu.get_xyz_gyro(gyro_angles);
	imu.get_xyz_gyro_dps(gyro_angles_dps);
	imu.get_xyz_accel(accel_values);
	imu.get_xyz_accel_mps2(accel_mps2);
	imu.get_xyz_mag(mag_values);
	
	
	Serial.print("gyro:\t");
	Serial.print(gyro_angles_dps[0]);
	Serial.print("\t");
	Serial.print(gyro_angles_dps[1]);
	Serial.print("\t");
	Serial.print(gyro_angles_dps[2]);
	Serial.print("\t");
	
	Serial.print("accel:\t");
	Serial.print(accel_mps2[0]);
	Serial.print("\t");
	Serial.print(accel_mps2[1]);
	Serial.print("\t");
	Serial.print(accel_mps2[2]);
	Serial.print("\t");
	
	Serial.print("mag:\t");
	Serial.print(mag_values[0]);
	Serial.print("\t");
	Serial.print(mag_values[1]);
	Serial.print("\t");
	Serial.println(mag_values[2]);
	
	delay(5);
}

void printCalibrationOfGyro(){
	int16_t gyro_angles[3];
	
	while(1){
		imu.read_xyz_gyro();
		imu.get_xyz_gyro(gyro_angles);
			
		Serial.print("gyro:\t");
		Serial.print(gyro_angles[0]);
		Serial.print("\t");
		Serial.print(gyro_angles[1]);
		Serial.print("\t");
		Serial.println(gyro_angles[2]);
	}
}

void printCalibrationOfAccelerometer(){
	int16_t accel_values[3];
	
	while(1){
		imu.read_xyz_accel();
		imu.get_xyz_accel(accel_values);
			
		Serial.print("accel:\t");
		Serial.print(accel_values[0]);
		Serial.print("\t");
		Serial.print(accel_values[1]);
		Serial.print("\t");
		Serial.println(accel_values[2]);
	}
}

void printCalibrationOfMagnetometer(){
	float mag_values[3];
	
	while(1){
		imu.read_xyz_mag();
		imu.get_xyz_mag(mag_values);
		
		Serial.print("mag:\t");
		Serial.print(mag_values[0]);
		Serial.print("\t");
		Serial.print(mag_values[1]);
		Serial.print("\t");
		Serial.println(mag_values[2]);
	}
}
