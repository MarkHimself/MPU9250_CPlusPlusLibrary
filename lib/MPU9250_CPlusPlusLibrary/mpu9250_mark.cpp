
#include "mpu9250_mark.h"


mpu9250_mark::mpu9250_mark(
	void (*cs_state_fn) (bool high), 
	unsigned char (*writeSPI1_fn) (uint8_t t),
	void (*delay_ms_fn)(uint32_t ms),
	void (*delay_us_fn)(uint32_t us)
){
	cs_state = cs_state_fn;
	writeSPI1 = writeSPI1_fn;
	delay_ms = delay_ms_fn;
	delay_us = delay_us_fn;
	
	// initialize variables
	
	for (uint8_t i = 0; i < 3; i++){								// init biases
		mag_bias[i] = 0;
		gyro_bias[i] = 0;
		accel_bias[i] = 0;
		accel_mult_bias[i] = 1;
	}
	
	gyro_cur_range = dps_250;										// default when gyro is powered on from off state
	accel_cur_range = g2;											// default when gyro is powered on from off state
	mag_is_setup = false;
	
	temperature = 0;
	TEMP_OUT = 0;
}

uint8_t mpu9250_mark::writeReg(uint8_t address, uint8_t data){
	uint8_t ret = 0;
	(*cs_state)(false);
	delay_us(10);
	writeSPI1(address);
	delay_us(5);
	ret = writeSPI1(data);
	cs_state(true);
	delay_us(100);
	return ret;
}

uint8_t mpu9250_mark::readReg(uint8_t address){
	return writeReg(address | READ_FLAG, 0x00);
}

void mpu9250_mark::readRegs(uint8_t address, uint8_t *buffer, uint8_t count){
	
	if (count == 0) return;
	cs_state(false);
	delay_us(5);
	writeSPI1(address | READ_FLAG);
	for (uint8_t i = 0; i < count; i++){
		buffer[i] = writeSPI1(0x00);
	}
	cs_state(true);
}

void mpu9250_mark::writeRegMag(uint8_t address, uint8_t data){
	
	uint8_t temp = 0;
	
	writeReg(I2C_SLV4_CTRL, 0x00);					// reset control register
	
	writeReg(USER_CTRL, USER_CTRL_I2C_MST_EN);		// unable i2c master mode, without this here, it wouldn't work
	
	writeReg(I2C_SLV4_ADDR, I2C_SLV4_ADDR_I2C_ID_4(0x0C));	// magnetometer address with a write request (no read bit)
	writeReg(I2C_SLV4_REG, address);				// request to write to memory address 0x0A (control register 1)
	writeReg(I2C_SLV4_DO, data);					// write data
	
	delay_ms(5);
	
	writeReg(I2C_SLV4_CTRL, I2C_SLV4_CTRL_I2C_SLV4_EN);	// enable the write transaction
	delay_ms(5);
	
	temp = readReg(I2C_MST_STATUS);
	writeReg(I2C_SLV4_CTRL, 0x00);
	
	while (!(	temp & I2C_MST_STATUS_I2C_SLV4_DONE)	){
		delay_us(50);
		temp = readReg(I2C_MST_STATUS);
	}	// wait for end of transaction
	while (readReg(I2C_SLV4_CTRL) & I2C_SLV4_CTRL_I2C_SLV4_EN){
		delay_us(50);
	}
}

uint8_t mpu9250_mark::readRegMag(uint8_t address){
	
	writeReg(I2C_SLV4_CTRL, 0x00);					// reset control register
	writeReg(I2C_SLV4_ADDR, 0x0C | READ_FLAG);		// magnetometer address with a read request
	writeReg(I2C_SLV4_REG, address);				// request to read memory address
	
	writeReg(I2C_SLV4_CTRL, I2C_SLV4_CTRL_I2C_SLV4_EN);					// enable the read transaction
	delay_ms(5);
	
	while (!(	readReg(I2C_MST_STATUS) & I2C_MST_STATUS_I2C_SLV4_DONE)	);	// wait for end of transaction
	while (readReg(I2C_SLV4_CTRL) & I2C_SLV4_CTRL_I2C_SLV4_EN); 
	return readReg(I2C_SLV4_DI);							// return read value
}

void mpu9250_mark::init(){									// do initialization...
	
	writeReg(PWR_MGMT_1, PWR_MGMT_1_H_RESET);				// perform power reset
	delay_ms(100);
	writeReg(SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_GYRO_RST | SIGNAL_PATH_RESET_ACCEL_RST | SIGNAL_PATH_RESET_TEMP_RST);		// reset gyro, accel and temp.
	delay_ms(100);
	
	writeReg(PWR_MGMT_2, readReg(PWR_MGMT_2) &				// ensure accelerometer and gyroscope is enabled
		 	~( 0
				| PWR_MGMT_2_DISABLE_ZG
				| PWR_MGMT_2_DISABLE_YG
				| PWR_MGMT_2_DISABLE_XG
				| PWR_MGMT_2_DISABLE_ZA
				| PWR_MGMT_2_DISABLE_YA
				| PWR_MGMT_2_DISABLE_XA
			)
		)
		;
	
	delay_ms(1);
	writeReg(PWR_MGMT_1, PWR_MGMT_1_CLKSEL_1);				// use x-gyro for clock (recommended)
	delay_ms(1);
	writeReg(FIFO_EN, 0x00);								// disable FIFO
	delay_ms(1);
	writeReg(SMPLRT_DIV, SMPLRT_DIV_SMPLRT_DIV(0x00));		// no sample rate dividor (full speed)
	delay_ms(1);
	writeReg(CONFIG, 0x00);									// don't use DLPF
	delay_ms(1);
	
	accel_16g_range();										// use +/- 16g for accel output
	gyro_2000dps_range();									// use +/- 2000 deg/s gyro output
}

void mpu9250_mark::init_mag(){						// MAGNETOMETER SETUP
	
	writeReg(I2C_SLV0_CTRL, 0x00);					// reset control register
	delay_ms(1);
	
	writeReg(USER_CTRL, USER_CTRL_I2C_MST_EN);		// unable i2c master mode
	delay_ms(1);
	
	writeReg(I2C_MST_CTRL, I2C_MST_CTRL_MULT_MST_EN | I2C_MST_CTRL_I2C_MST_CLK_13);				// mulit-master enable, use 400 KHz
	delay_ms(1);
	
	writeRegMag(AK8963_CNTL1, AK8963_CNTL1_POWERDOWN);		// power down mag
	delay_ms(10);
	writeRegMag(AK8963_CNTL2, AK8963_CNTL2_SRST);			// soft reset of mag
	delay_ms(10);
	
	get_H_adj();
	mag_mode_2();
	setup_mag_readings_automatically();
	mag_is_setup = true;
}

void mpu9250_mark::get_H_adj(){
	uint8_t temp_buf[3];
	float temp = 0;
	
	writeRegMag(AK8963_CNTL1, AK8963_CNTL1_16BIT | AK8963_CNTL1_FUSEROM);		// enter fuse ROM access mode
	delay_ms(10);
	for (int8_t i = 0; i < 3; i++) temp_buf[i] = readRegMag(AK8963_ASAX + i);	// read the 3 registers
	writeRegMag(AK8963_CNTL1, AK8963_CNTL1_16BIT | AK8963_CNTL1_POWERDOWN);		// power down mag
	delay_us(10);
	for (int8_t i = 0; i < 3; i++) Hadj_factor[i] = (((float) (temp_buf[i]) - 128) / 256) + 1;
	
	temp = Hadj_factor[0];									// switch x and y to match mpu9250 gyro/accel orientation
	Hadj_factor[0] = Hadj_factor[1];
	Hadj_factor[1] = temp;
}

void mpu9250_mark::mag_mode_2(){
	writeRegMag(AK8963_CNTL1, AK8963_CNTL1_16BIT | AK8963_CNTL1_MODE2);	// use 16-bit values and put to mode 2
	delay_ms(1);
}

void mpu9250_mark::setup_mag_readings_automatically(){	// mpu9250 will continuously read from magnetomer and save data in EX_SENS_DATA_00-06
	writeReg(I2C_SLV0_CTRL, 0x00);						// reset control register
	delay_ms(1);
	
	writeReg(I2C_SLV0_ADDR, 0x0C | READ_FLAG);			// will read from magnetometer
	writeReg(I2C_SLV0_REG, AK8963_HXL);					// start reading from HXL
	writeReg(I2C_SLV0_CTRL, I2C_SLV0_CTRL_I2C_SLV0_LENG(7));	// set for reading 7 bytes
	writeReg(I2C_SLV0_CTRL, I2C_SLV0_CTRL_I2C_SLV0_EN | I2C_SLV0_CTRL_I2C_SLV0_LENG(7));	// enable reading magnetometer and read 7 bytes
}

uint8_t mpu9250_mark::whoAmI(){						// should be 115
	uint8_t temp = 0;
	temp = readReg(WHO_AM_I);
	return temp;
}


void mpu9250_mark::readAll(){
	
	uint8_t temp_buf[14] = {0};							// {6 bytes of accel data, 2 bytes of temp data, 6 bytes of gyro data}
	int16_t received_accel_data[3];
	int16_t received_gyro_data[3];
	
	readRegs(ACCEL_XOUT_H, temp_buf, 14);				// read accel, temp, gyro data
	
	for (int8_t i = 0; i < 3; i++){						// get accel data
		received_accel_data[i] = (temp_buf[i*2] << 8) | temp_buf[i*2 + 1];
	}
	update_x_accel(received_accel_data[0]);
	update_y_accel(received_accel_data[1]);
	update_z_accel(received_accel_data[2]);
	
	TEMP_OUT = (temp_buf[6] << 8) | temp_buf[7];		// get temp data
	update_temp();
	
	for (int8_t i = 0; i < 3; i++){						// get gyro data
		received_gyro_data[i] = (temp_buf[i*2 + 8] << 8) | temp_buf[i*2 + 1 + 8];
	}
	update_x_gyro(received_gyro_data[0]);
	update_y_gyro(received_gyro_data[1]);
	update_z_gyro(received_gyro_data[2]);
	
	if (mag_is_setup){									// get magnetometer data
		delay_us(10);
		read_xyz_mag();
	}
}


// accelerometer methods

void mpu9250_mark::accel_2g_range(){
	writeReg(ACCEL_CONFIG, ACCEL_CONFIG_2G);
	accel_cur_range = g2;
}
void mpu9250_mark::accel_4g_range(){
	writeReg(ACCEL_CONFIG, ACCEL_CONFIG_4G);
	accel_cur_range = g4;
}
void mpu9250_mark::accel_8g_range(){
	writeReg(ACCEL_CONFIG, ACCEL_CONFIG_8G);
	accel_cur_range = g8;
}
void mpu9250_mark::accel_16g_range(){
	writeReg(ACCEL_CONFIG, ACCEL_CONFIG_16G);
	accel_cur_range = g16;
}

void mpu9250_mark::read_xyz_accel(){
	uint8_t temp_buf[6] = {0};
	int16_t received_accel_data[3];
	readRegs(ACCEL_XOUT_H, temp_buf, 6);
	
	for (int8_t i = 0; i < 3; i++){
		received_accel_data[i] = (temp_buf[i*2] << 8) | temp_buf[i*2 + 1];
	}
	
	update_x_accel(received_accel_data[0]);
	update_y_accel(received_accel_data[1]);
	update_z_accel(received_accel_data[2]);
}

void mpu9250_mark::read_x_accel(){
	uint8_t buf[2] = {0};
	uint16_t ret = 0;
	readRegs(ACCEL_XOUT_H, buf, 2);
	ret = buf[0] << 8;
	ret |= buf[1];
	update_x_accel(ret);
}

void mpu9250_mark::read_y_accel(){
	uint8_t buf[2] = {0};
	uint16_t ret = 0;
	readRegs(ACCEL_YOUT_H, buf, 2);
	ret = buf[0] << 8;
	ret |= buf[1];
	update_y_accel(ret);
}

void mpu9250_mark::read_z_accel(){
	uint8_t buf[2] = {0};
	uint16_t ret = 0;
	readRegs(ACCEL_ZOUT_H, buf, 2);
	ret = buf[0] << 8;
	ret |= buf[1];
	update_z_accel(ret);
}

void mpu9250_mark::get_xyz_accel(int16_t *buf){
	buf[0] = x_accel;
	buf[1] = y_accel;
	buf[2] = z_accel;
}
int16_t mpu9250_mark::get_x_accel(){
	return x_accel;
}
int16_t mpu9250_mark::get_y_accel(){
	return y_accel;
}
int16_t mpu9250_mark::get_z_accel(){
	return z_accel;
}

void mpu9250_mark::get_xyz_accel_mps2(float *buf){						// get accel in meters per second
	buf[0] = get_x_accel_mps2();
	buf[1] = get_y_accel_mps2();
	buf[2] = get_z_accel_mps2();
}
float mpu9250_mark::get_x_accel_mps2(){
	float temp;
	if (accel_cur_range == g2) 		temp = (float) x_accel * 2.0 * 9.81 / 32768.0;
	else if (accel_cur_range == g4) temp = (float) x_accel * 4.0 * 9.81 / 32768.0;
	else if (accel_cur_range == g8) temp = (float) x_accel * 8.0 * 9.81 / 32768.0;
	else 							temp = (float) x_accel * 16.0 * 9.81 / 32768.0;
	return temp;
}
float mpu9250_mark::get_y_accel_mps2(){
	float temp;
	if (accel_cur_range == g2) 		temp = (float) y_accel * 2.0 * 9.81 / 32768.0;
	else if (accel_cur_range == g4) temp = (float) y_accel * 4.0 * 9.81 / 32768.0;
	else if (accel_cur_range == g8) temp = (float) y_accel * 8.0 * 9.81 / 32768.0;
	else 							temp = (float) y_accel * 16.0 * 9.81 / 32768.0;
	return temp;
}
float mpu9250_mark::get_z_accel_mps2(){
	float temp;
	if (accel_cur_range == g2) 		temp = (float) z_accel * 2.0 * 9.81 / 32768.0;
	else if (accel_cur_range == g4) temp = (float) z_accel * 4.0 * 9.81 / 32768.0;
	else if (accel_cur_range == g8) temp = (float) z_accel * 8.0 * 9.81 / 32768.0;
	else 							temp = (float) z_accel * 16.0 * 9.81 / 32768.0;
	return temp;
}

void mpu9250_mark::update_x_accel(int16_t received){
	x_accel = (int16_t) (accel_mult_bias[0] * (float)(received - accel_bias[0]));
}
void mpu9250_mark::update_y_accel(int16_t received){
	y_accel = (int16_t) (accel_mult_bias[1] * (float)(received - accel_bias[1]));
}
void mpu9250_mark::update_z_accel(int16_t received){
	z_accel = (int16_t) (accel_mult_bias[2] * (float)(received - accel_bias[2]));
}

void mpu9250_mark::update_accel_bias(int16_t x, int16_t y, int16_t z){			// Use raw accel readings to update bias
	accel_bias[0] = x;
	accel_bias[1] = y;
	accel_bias[2] = z;
}
void mpu9250_mark::update_x_accel_bias(int16_t xbias){
	accel_bias[0] = xbias;
}
void mpu9250_mark::update_y_accel_bias(int16_t ybias){
	accel_bias[1] = ybias;
}
void mpu9250_mark::update_z_accel_bias(int16_t zbias){
	accel_bias[2] = zbias;
}

void mpu9250_mark::update_accel_mult_bias(float x, float y, float z){				// Use raw accel readings to update multiplicative bias
	accel_mult_bias[0] = x;
	accel_mult_bias[1] = y;
	accel_mult_bias[2] = z;
}
void mpu9250_mark::update_x_accel_mult_bias(float xbias){
	accel_mult_bias[0] = xbias;
}
void mpu9250_mark::update_y_accel_mult_bias(float ybias){
	accel_mult_bias[1] = ybias;
}
void mpu9250_mark::update_z_accel_mult_bias(float zbias){
	accel_mult_bias[2] = zbias;
}


// Gyro methods

void mpu9250_mark::gyro_250dps_range(){
	writeReg(GYRO_CONFIG, GYRO_CONFIG_250DPS);
	gyro_cur_range = dps_250;
}
void mpu9250_mark::gyro_500dps_range(){
	writeReg(GYRO_CONFIG, GYRO_CONFIG_500DPS);
	gyro_cur_range = dps_500;
}
void mpu9250_mark::gyro_1000dps_range(){
	writeReg(GYRO_CONFIG, GYRO_CONFIG_1000DPS);
	gyro_cur_range = dps_1000;
}
void mpu9250_mark::gyro_2000dps_range(){
	writeReg(GYRO_CONFIG, GYRO_CONFIG_2000DPS);
	gyro_cur_range = dps_2000;
}

void mpu9250_mark::read_xyz_gyro(){		// read gyro data, update it into actual data, return it
	uint8_t temp_buf[6] = {0};
	int16_t received_gyro_data[3];
	
	readRegs(GYRO_XOUT_H, temp_buf, 6);
	
	for (int8_t i = 0; i < 3; i++){
		received_gyro_data[i] = (temp_buf[i*2] << 8) | temp_buf[i*2 + 1];
	}
	
	update_x_gyro(received_gyro_data[0]);
	update_y_gyro(received_gyro_data[1]);
	update_z_gyro(received_gyro_data[2]);
}

// read the latest gyro axis. // update it with corrections. return value
void mpu9250_mark::read_x_gyro(){
	uint8_t buf[2] = {0};
	uint16_t ret = 0;
	readRegs(GYRO_XOUT_H, buf, 2);
	ret = buf[0] << 8;
	ret |= buf[1];
	update_x_gyro(ret);
}
void mpu9250_mark::read_y_gyro(){
	uint8_t buf[2] = {0};
	uint16_t ret = 0;
	readRegs(GYRO_YOUT_H, buf, 2);
	ret = buf[0] << 8;
	ret |= buf[1];
	update_y_gyro(ret);
}
void mpu9250_mark::read_z_gyro(){
	uint8_t buf[2] = {0};
	uint16_t ret = 0;
	readRegs(GYRO_ZOUT_H, buf, 2);
	ret = buf[0] << 8;
	ret |= buf[1];
	update_z_gyro(ret);
}

void mpu9250_mark::update_x_gyro(int16_t received){
	x_gyro = received - gyro_bias[0];
}
void mpu9250_mark::update_y_gyro(int16_t received){
	y_gyro = received - gyro_bias[1];
}
void mpu9250_mark::update_z_gyro(int16_t received){
	z_gyro = received - gyro_bias[2];
}

void mpu9250_mark::get_xyz_gyro(int16_t *buf){
	buf[0] = x_gyro;
	buf[1] = y_gyro;
	buf[2] = z_gyro;
}

int16_t mpu9250_mark::get_x_gyro(){									// return the raw gyro value (-32768 to 32767)
	return x_gyro;
}
int16_t mpu9250_mark::get_y_gyro(){
	return y_gyro;
}
int16_t mpu9250_mark::get_z_gyro(){
	return z_gyro;
}

void mpu9250_mark::get_xyz_gyro_dps(float *buf){
	buf[0] = get_x_gyro_dps();
	buf[1] = get_y_gyro_dps();
	buf[2] = get_z_gyro_dps();
}

float mpu9250_mark::get_x_gyro_dps(){			// get the value in degrees per second
	float temp;
	if (gyro_cur_range == dps_250) 			temp = (float) x_gyro * 250.0 / 32768.0;
	else if (gyro_cur_range == dps_500) 	temp = (float) x_gyro * 500.0 / 32768.0;
	else if (gyro_cur_range == dps_1000) 	temp = (float) x_gyro * 1000.0 / 32768.0;
	else 									temp = (float) x_gyro * 2000.0 / 32768.0;
	return temp;
}
float mpu9250_mark::get_y_gyro_dps(){
	float temp;
	if (gyro_cur_range == dps_250) 			temp = (float) y_gyro * 250.0 / 32768.0;
	else if (gyro_cur_range == dps_500) 	temp = (float) y_gyro * 500.0 / 32768.0;
	else if (gyro_cur_range == dps_1000) 	temp = (float) y_gyro * 1000.0 / 32768.0;
	else 									temp = (float) y_gyro * 2000.0 / 32768.0;
	return temp;
}
float mpu9250_mark::get_z_gyro_dps(){
	float temp;
	if (gyro_cur_range == dps_250) 			temp = (float) z_gyro * 250.0 / 32768.0;
	else if (gyro_cur_range == dps_500) 	temp = (float) z_gyro * 500.0 / 32768.0;
	else if (gyro_cur_range == dps_1000) 	temp = (float) z_gyro * 1000.0 / 32768.0;
	else 									temp = (float) z_gyro * 2000.0 / 32768.0;
	return temp;
}

// bias is based on value read from mpu9250 register. (-32768 to 32767) and not degrees per second or whatever...
void mpu9250_mark::update_gyro_bias(int16_t x, int16_t y, int16_t z){
	gyro_bias[0] = x;
	gyro_bias[1] = y;
	gyro_bias[2] = z;
}
void mpu9250_mark::update_x_gyro_bias(int16_t xbias){
	gyro_bias[0] = xbias;
}
void mpu9250_mark::update_y_gyro_bias(int16_t ybias){
	gyro_bias[1] = ybias;
}
void mpu9250_mark::update_z_gyro_bias(int16_t zbias){
	gyro_bias[2] = zbias;
}



// Magnetometer methods

// when read x, actually get y
// when read y, actually get x
// when read z, actually get -z

void mpu9250_mark::read_xyz_mag(){
	uint8_t temp_buf[7] = {0};
	int16_t received_mag_data[3];
	
	readRegs(EXT_SENS_DATA_00, temp_buf, 7);					// using mpu external data
	
	for (int8_t i = 0; i < 3; i++){
		received_mag_data[i] = (temp_buf[i*2 + 1] << 8) | temp_buf[i*2];
	}
	update_x_mag(received_mag_data[1]);							// correct the data (multiplicative factor and offset)
	update_y_mag(received_mag_data[0]);
	update_z_mag(-received_mag_data[2]);
	
}

float mpu9250_mark::read_x_mag(){
	uint8_t temp_buf[2] = {0};
	int16_t x_initial = 0;
	
	temp_buf[0] = readRegMag(AK8963_HYL);
	temp_buf[1] = readRegMag(AK8963_HYH);
	readRegMag(AK8963_ST2);
	
	x_initial = (temp_buf[1] << 8) | temp_buf[0];
	update_x_mag(x_initial);
	return x_mag;
}

float mpu9250_mark::read_y_mag(){
	uint8_t temp_buf[2] = {0};
	int16_t y_initial = 0;
	
	temp_buf[0] = readRegMag(AK8963_HXL);
	temp_buf[1] = readRegMag(AK8963_HXH);
	readRegMag(AK8963_ST2);
	
	y_initial = (temp_buf[1] << 8) | temp_buf[0];
	update_y_mag(y_initial);
	return y_mag;
}

float mpu9250_mark::read_z_mag(){
	uint8_t temp_buf[2] = {0};
	int16_t z_initial = 0;
	
	temp_buf[0] = readRegMag(AK8963_HZL);
	temp_buf[1] = readRegMag(AK8963_HZH);
	readRegMag(AK8963_ST2);
	
	z_initial = (temp_buf[1] << 8) | temp_buf[0];
	update_z_mag(-z_initial);
	return z_mag;
}

void mpu9250_mark::update_x_mag(int16_t received){
	x_mag = Hadj_factor[0] * received - mag_bias[0];
}
void mpu9250_mark::update_y_mag(int16_t received){
	y_mag = Hadj_factor[1] * received - mag_bias[1];
}
void mpu9250_mark::update_z_mag(int16_t received){
	z_mag = Hadj_factor[2] * received - mag_bias[2];
}

void mpu9250_mark::get_xyz_mag(float buf[3]){
	
	buf[0] = get_x_mag();										// place correct magnetometer readings into the buffer
	buf[1] = get_y_mag();
	buf[2] = get_z_mag();
}

float mpu9250_mark::get_x_mag(){
	return x_mag;
}
float mpu9250_mark::get_y_mag(){
	return y_mag;
}
float mpu9250_mark::get_z_mag(){
	return z_mag;
}

void mpu9250_mark::update_mag_bias(float x, float y, float z){
	mag_bias[0] = x;
	mag_bias[1] = y;
	mag_bias[2] = z;
}

void mpu9250_mark::update_x_mag_bias(float xbias){
	mag_bias[0] = xbias;
}
void mpu9250_mark::update_y_mag_bias(float ybias){
	mag_bias[1] = ybias;
}
void mpu9250_mark::update_z_mag_bias(float zbias){
	mag_bias[2] = zbias;
}

void mpu9250_mark::get_xyz_mag_adj_factor(float adj[3]){
	for (int i = 0; i < 3; i++){
		adj[i] = Hadj_factor[i];
	}
}

void mpu9250_mark::set_xyz_mag_adj_factor(float xAdj, float yAdj, float zAdj){
	Hadj_factor[0] = xAdj;
	Hadj_factor[1] = yAdj;
	Hadj_factor[2] = zAdj;
}


uint8_t mpu9250_mark::device_id_mag(){				// magnetometer device ID; 0x48 (72 decimal)
	uint8_t temp = 0;
	temp = readRegMag(AK8963_WIA);
	return temp;
}


// Temperature Methods

void mpu9250_mark::read_temperature(){
	uint8_t buf[2] = {0};
	readRegs(TEMP_OUT_H, buf, 2);
	TEMP_OUT = (buf[0] << 8) | buf[1];
	update_temp();
}

uint16_t mpu9250_mark::get_raw_temp_out(){
	return TEMP_OUT;
}

float mpu9250_mark::get_temp(){
	return temperature;
}

void mpu9250_mark::update_temp(){
	temperature = ((TEMP_OUT - 0)/321) + 21;
}



// ************************** START OF Brians bolderflight mpu9250 code BELOW **************************** //
// I'm using Brians bolderflight mpu9250 code to initialize the mpu9250 due to its robustness in freeing an I2C hang on the magnetometer.


// This method is from Brians bolderflight mpu9250 code available on GitHub
// writes a register to the AK8963 given a register address and data
int mpu9250_mark::writeAK8963Register(uint8_t subAddress, uint8_t data){
	
	if (writeRegister(I2C_SLV0_ADDR, 0x0C) < 0) {					// set slave 0 to the AK8963 and set for write
		return -1;
	}
	
	if (writeRegister(I2C_SLV0_REG, subAddress) < 0) {				// set the register to the desired AK8963 sub address 
		return -2;
	}
	
	if (writeRegister(I2C_SLV0_DO, data) < 0) {						// store the data for write
		return -3;
	}
	
	if (writeRegister(I2C_SLV0_CTRL, 0x80 | (uint8_t)1) < 0) {		// enable I2C and send 1 byte
		return -4;
	}
  
	if (readAK8963Registers(subAddress, 1, _buffer) < 0) {			// read the register and confirm
		return -5;
	}
	
	if (_buffer[0] == data) {
		return 1;
	} else {
		return -6;
	}
}

// This method is from Brians bolderflight mpu9250 code available on GitHub
//reads registers from the AK8963
int mpu9250_mark::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
	
	if (writeRegister(I2C_SLV0_ADDR, 0x0C | READ_FLAG) < 0) {			// set slave 0 to the AK8963 and set for read
		return -1;
	}
	
	if (writeRegister(I2C_SLV0_REG, subAddress) < 0) {				// set the register to the desired AK8963 sub address
		return -2;
	}
	
	if (writeRegister(I2C_SLV0_CTRL, 0x80 | count) < 0) {				// enable I2C and request the bytes
		return -3;
	}
	delay_us(10); // takes some time for these registers to fill
	
	int _status = readRegisters(EXT_SENS_DATA_00, count, dest); 		// read the bytes off the MPU9250 EXT_SENS_DATA registers
	return _status;
}


// This method is from Brians bolderflight mpu9250 code available on GitHub
/* writes a byte to MPU9250 register given a register address and data */
int mpu9250_mark::writeRegister(uint8_t subAddress, uint8_t data){
	/* write data to device */
	uint8_t buf[0];
	
	writeReg(subAddress, data);
	delay_us(10);
	readRegisters(subAddress, 1, buf);				// read back the register
	
	if (buf[0] == data) {							// check the read back register against the written register
		return 1;
	}
	else {
		return -1;
	}
}

// This method is from Brians bolderflight mpu9250 code available on GitHub
/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
int mpu9250_mark::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
	// begin the transaction
	cs_state(false);						//// select the MPU9250 chip
	delay_us(10);
	writeSPI1(subAddress | READ_FLAG);		// specify the starting register address
	for(uint8_t i = 0; i < count; i++){
		dest[i] = writeSPI1(0x00);			//// read the data
	}
	cs_state(true);							// deselect the MPU9250 chip
	delay_us(10);
	return 1;
}

// This method is from Brians bolderflight mpu9250 code available on GitHub
int8_t mpu9250_mark::init_brian(){
	
	writeRegister(PWR_MGMT_1, 0x80);
	delay_ms(100);
	
	if(writeRegister(PWR_MGMT_1, 0x01) < 0){					// select clock source to gyro
		return -1;
	}
	
	if(writeRegister(USER_CTRL, 0x20) < 0){						// enable I2C master mode
		return -2;
	}
	
	if(writeRegister(I2C_MST_CTRL, 0x0D) < 0){					// set the I2C bus speed to 400 kHz
		return -3;
	}
	
	writeAK8963Register(AK8963_CNTL1, 0x00);					// set AK8963 to Power Down	
	writeRegister(PWR_MGMT_1, 0x80);							// reset the MPU9250
	delay_ms(100);													// wait for MPU-9250 to come back up
	writeAK8963Register(AK8963_CNTL2, 0x01);					// reset the AK8963
	delay_ms(100);
	
	if(writeRegister(USER_CTRL, 0x20) < 0){						// enable I2C master mode
		return -12;
	}
	
	if( writeRegister(I2C_MST_CTRL, 0x0D) < 0){					// set the I2C bus speed to 400 kHz
		return -13;
	}
	delay_us(1);
	int tempReturn = 0;
	
	if( (tempReturn = whoAmIAK8963()) != 72 ){									// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
		return -14;
	}
	
	delay_us(3);
	
	if(writeAK8963Register(AK8963_CNTL1, 0x00) < 0){			// set AK8963 to Power Down
		return -15;
	}
	
	writeAK8963Register(AK8963_CNTL2, 0x01);			// soft reset mag
	writeAK8963Register(AK8963_CNTL1, 0x00);			// power down mag
	
	writeReg(I2C_SLV0_CTRL, 0x00);						// disable reading more data
	uint8_t temp_buf[10];
	readRegs(EXT_SENS_DATA_00, temp_buf, 10);			// read/clear data, if any
	writeReg(PWR_MGMT_1, 0x80);							// perform power reset
	
	return 1;											// successful init, return 1
}

// This method is from Brians bolderflight mpu9250 code available on GitHub
// gets the AK8963 WHO_AM_I register value, expected to be 0x48
int mpu9250_mark::whoAmIAK8963(){
	
	if (readAK8963Registers(0x00, 1, _buffer) < 0) {		// read the WHO AM I register
		return -1;
	}
	return _buffer[0];										// return the register value
}
