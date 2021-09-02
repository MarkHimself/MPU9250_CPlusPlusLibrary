
The MPU9250 library consists of the files:
- mpu9250_mark.cpp
- mpu9250_mark.h
copy these files to wherever where you need them...
(i.e. if using Arduino framework, they will go in the Documents\Arduino\libraries\mpu9250_mark\ folder)
The other files are used for the Arduino Due board (SAM3X8E microcontroller) to interface with the MPU9250 through the library.

Note to Arduino users:
The src & include directories are being used here because I've used atom text editor & PlatformIO and not Arduino IDE.
My examples use low level initialization of SPI and not the Arduino versions.
If you want to use the Arduino versions of SPI, you will need to wrap the SPI object and the write method SPI.transfer() into a function and pass the function into the MPU9250 object.

Note to everyone else:
this library should be independant of Arduino.
The basic functions that the library relies on (SPI write, toggling CS line & delays) are passed into the library object through function pointers.
This should allow any framework or bare metal code to use the library.

What is the MPU9250?
It is an Accelerometer & Gyroscope sensor (MPU6500) that can also communicate with an internal magnetometer (AK8963) via I2C.

This library only supports SPI communication between the host microcontroller and the MPU9250 sensor.

This MPU9250 communicates with the internal magnetometer (AK8963) via I2C (if set up to do so).
This MPU9250 library contains the necessary code to setup the magnetometer.
It works by instructing the MPU9250 to read the registers in the AK8963 and then save them in the MPU9250.
However, an I2C hang can occur.
This will most likely occur when the host MCU is reset (watchdog reset or press a button to reset it).
A reset can occur while the MPU9250 was reading data from the magnetometer.
The MCU reset will then proceed to initialize the MPU9250 and the initialization may lead to an I2C bus hang.
I've found that the initialization code from Brian's/Bolderflight MPU9250 library was pretty good at avoiding I2C bus hangs.
I decided to use his initialization code in my library and afterwards proceed to my initialization and library code.


// *** Basic Library Usage *** \\

// Make the MPU9250 object (and make it global):
mpu9250_mark imu(spi_cs_imu, spi_send, delay, delayMicroseconds);

// The expected function prototypes are:
void spi_cs_imu_due(bool high);
uint8_t spi_send_due(uint8_t t);
void delay_milliseconds(uint32_t ms);
void delay_microseconds(uint32_t us);


// Configure the sensor with the following code:
// This code attempts to initialize the gyro/accelerometer and the magnetometer.
// it will loop until it succeeds.
	do{
		int init_val;
		while ((init_val = imu.init_brian()) <= 0){
			Serial.println("failed to restart mpu");
		}
		Serial.println("The return status of brians init is:...");
		Serial.println(init_val);
		imu.init();
		imu.init_mag();
	} while(imu.device_id_mag() != 72 || imu.whoAmI() != 115 );
	// update the biases (see below section on calibration instructions)
	imu.update_accel_bias(0, 0, 0);
	imu.update_accel_mult_bias(1, 1, 1);
	imu.update_gyro_bias(0, 0, 0);
	imu.update_mag_bias(0, 0, 0);
	imu.set_xyz_mag_adj_factor(1, 1, 1);
	
	// update the gyro and accel ranges. Use what you need.
	imu.gyro_2000dps_range();
	imu.accel_8g_range();
	
	// read all the values (gyro, accelerometer & magnetometer)
	imu.readAll();
	
	// get the last read values
	imu.get_xyz_gyro_dps(gyro_angles_dps);
	imu.get_xyz_accel_mps2(accel_mps2);
	imu.get_xyz_mag(mag_values);

// For more usage methods, read the mpu9250_mark.h header file.

// *** Calibration *** \\
As expected, the gyroscope, accelerometer and magnetometer all need to be calibrated.
Every sensor is slightly different. This means my calibration values may not necessarily work for you.
Also, you CANNOT calibrate the sensor "on the fly" (Just because there are MPU9250 libraries that calibrate the sensor during startup does not mean that this is OK to do)
Why? Okay you can. ...but the calibration biases will be WRONG.
The correct procedure for calibrating the sensor will be discussed below.
The biases for each measurement will then be hard-coded into the mcu code and used with the corresponding sensor.
i.e. the calibration biases do not change when power is cycled (so you can continue using them)
I recommend copying the data into excel and performing the math in it because all the readings (i typically use 5-10 thousand samples) can be checked for any outliers.
After calibrating the magnetometer, gyroscope and accelerometer... you will clearly see why you cannot run a calibration function whenever your drone resets in midair or something else (depending on application).


// Perform magnetometer calibration
1. 	Begin by uploading the following bias values:
	imu.update_mag_bias(0, 0, 0);
	imu.set_xyz_mag_adj_factor(1, 1, 1);
2. 	Read magnetometer values while waving mpu9250 in circles (wave it through at least 10 circles and around every axis).
	Try to stay away from sources of electricity (USB cables) as they can affect the magnetometer values.
	Waving the sensor and reading the magnetometer values will allow you to collect the max/min value of the magnetic field strength that each axis measures. 
2. 	Copy data into excel. You will need the x, y, z components of the magnetic field strength.
3. 	Find max value of each axis (you may ignore outliers) (use excel's min function)
4. 	Find min value of each axis (you may ignore outliers) (use excel's max function)
5. 	Take average of max and min value:
	average = ((max + min) / 2)
6. 	Take difference of max and min values: difference = (max - min)
7. 	Update mag_bias with the average value of each axis.
8. 	example: x-axis of magnetometer. 
	Min_x_axis = 86, max_x_axis = 671
	average_x_axis = ((86 + 671) / 2) = 378.5 = 379
	difference_x_axis = 671 - 86 = 585
	imu.update_mag_bias(379, ..., ...);
9.	Note that the difference between the x, y, z axes are different. We need to scale them to make them equal.
	Let's scale the smaller differences to match the axes with the largest difference.
	This is our adjustment factor
10.	adj_factor_for_current_axis = (largest_difference) / (difference_for_current_axis)
11.	Proceeding from the previous example, my differences for each axis were:
	x-axis: 585
	y-axis: 577
	z-axis: 546
	
	The largest difference is on the x-axis (585)
	adj_factor_for_x_axis = 1
	adj_factor_for_7_axis = 585 / 577 = 1.013865
	adj_factor_for_z_axis = 585 / 546 = 1.071429
12.	Hard-code the adjustment factors into the code
	imu.set_xyz_mag_adj_factor(1, 1.013865, 1.071429);
13.	Verify your results. If you repeat these steps but leave the biases (and adjustment factors) with the values you have found...
	The new data should have the average value for each axis very close to 0 and the adustment factors should be very close to 1.


// Perform gyro calibration
1. 	Begin by uploading the following bias values:
	imu.update_gyro_bias(0, 0, 0);
2. 	Read raw gyro values. Keep mpu9250 still! DO NOT MOVE!
	Use the following method to get the raw gyroscope output for the x, y and z axes:
	imu.get_xyz_gyro(buffer);
2. 	Copy data into excel. You will need the x, y, z components of the raw gyroscope output.
3. 	Find average of each axis
	average = ((max + min) / 2)
4. 	Update gyro_bias values using this method:
	imu.update_gyro_bias(0, 0, 0);
Scaling - Currently, library contains only default implementation.
1. gyro has built-in(factory programmed) scaling. can over-ride, if you want.
2. size_x = max_x - min_x, size_y = max_y - min_y, size_z = max_z - min_z, 
3. scale_x = size_x / size_x = 1, scale_y = size_x / size_y, scale_z = size_x / size_z
4. gyro values are computed as: (gyro_value - bias) * scale


// Perform accelerometer calibration
1. 	This process is similar to the magnetometer calibration.
	Except that if you wave the sensor, the accelerometer will read a bunch of noise.
	You will need to grab as many measuring points as feasible and perform the same math as in the instructions for calibrating the magnetometer.
	Here are the following methods that update the bias's
	imu.update_accel_bias(0, 0, 0);			// these biases (x, y, z) ensure the data is centered on 0
	imu.update_accel_mult_bias(1, 1, 1);	// these biases (x, y, z) ensure each axis measures the same values as the other ones.
	For simplicity, you may attempt to grab the smallest and largest values by pointing the sensor axes down/up, hold it still and record those values. (I use excel)
	Also, read the raw data (and not the meters per second squared)
	imu.get_xyz_accel(buffer);


// *** Wiring for SPI *** \\
MPU9250      |      MCU             |  		Arduino Due Example
VCC                 3.3V               		3.3V
GND                 GND                		GND
SCL/SCLK:           SPI Clock          		D76 SCK   (PIOA27) (On SPI header)
SDA/SDI:            MOSI               		D75 MOSI  (PIOA26) (On SPI header)
EDA:                Not Wired          		Not Wired
ECL:                Not Wired          		Not Wired
ADO/SDO:            MISO               		D74 MISO  (PIOA25) (On SPI header)
INT:                Not Wired          		Not Wired
NCS:                SPI Chip Select    		D10       (PIOA28)
FSYNC:              Not Wired          		Not Wired


// *** Further Documentation *** \\
Please consult the MPU9250 datasheet & register mapping. Also, consult the AK8963 datasheet.

