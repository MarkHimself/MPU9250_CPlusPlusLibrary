
// library for MPU9250
// SPI is used for talking with it.
// USE MODE 0 or 3!!! I prefer mode 0
// Mode 0 and Mode 3 both worked on Arduino Uno and Arduino Due

// todo:
// add method of using slow and high speed SPI
// add function pointers for delay functions (to make independant of particular mcu/framework)


#ifndef MPU9250_MARK_H
#define MPU9250_MARK_H
#include "stdint.h"


// ********************** MPU 9250 Registers ******************************** //

#define SMPLRT_DIV				0x19	//25
#define SMPLRT_DIV_SMPLRT_DIV(value)	((value) & 0xFF)
#define CONFIG					0x1A	//26
#define GYRO_CONFIG				0x1B	//27
#define GYRO_CONFIG_250DPS		0x00
#define GYRO_CONFIG_500DPS		(0x01 << 3)
#define GYRO_CONFIG_1000DPS		(0x02 << 3)
#define GYRO_CONFIG_2000DPS		(0x03 << 3)

#define ACCEL_CONFIG			0x1C	//28
#define ACCEL_CONFIG_2G			(0x00 << 3)
#define ACCEL_CONFIG_4G			(0x01 << 3)
#define ACCEL_CONFIG_8G			(0x02 << 3)
#define ACCEL_CONFIG_16G		(0x03 << 3)


#define FIFO_EN					0x23	//35
#define I2C_MST_CTRL			0x24	//36
#define I2C_MST_CTRL_MULT_MST_EN	(0x01 << 7)
#define I2C_MST_CTRL_I2C_MST_CLK_13	(13)
#define I2C_SLV0_ADDR			0x25	//37
#define I2C_SLV0_REG			0x26	//38
#define I2C_SLV0_CTRL			0x27	//39
#define I2C_SLV0_CTRL_I2C_SLV0_EN	(0x01 << 7)
#define I2C_SLV0_CTRL_I2C_SLV0_LENG(value)	((value) & 0x0F)

#define I2C_SLV3_ADDR			0x2E	//46
#define I2C_SLV3_REG			0x2F	//47
#define I2C_SLV3_CTRL			0x30	//48


#define I2C_SLV4_ADDR			0x31	//49
#define I2C_SLV4_ADDR_I2C_ID_4(value)	((value) & 0x7F)
#define I2C_SLV4_REG			0x32	//50
#define I2C_SLV4_DO				0x33	//51
#define I2C_SLV4_CTRL			0x34	//52
#define I2C_SLV4_CTRL_I2C_SLV4_EN	(0x01 << 7)
#define I2C_SLV4_DI				0x35	//53

#define I2C_MST_STATUS			0x36	//54
#define I2C_MST_STATUS_I2C_SLV4_DONE	(0x01 << 6)

#define INT_PIN_CFG				0x37	//55

#define ACCEL_XOUT_H			0x3B	//59
#define ACCEL_XOUT_L			0x3C
#define ACCEL_YOUT_H			0x3D
#define ACCEL_YOUT_L			0x3E
#define ACCEL_ZOUT_H			0x3F
#define ACCEL_ZOUT_L			0x40

#define TEMP_OUT_H				0x41
#define TEMP_OUT_L				0x42

#define GYRO_XOUT_H				0x43	// 67
#define GYRO_XOUT_L				0x44
#define GYRO_YOUT_H				0x45
#define GYRO_YOUT_L				0x46
#define GYRO_ZOUT_H				0x47
#define GYRO_ZOUT_L				0x48

#define EXT_SENS_DATA_00		0x49	//73

#define I2C_SLV0_DO				0x63	//99


#define SIGNAL_PATH_RESET		0x68	//104
#define SIGNAL_PATH_RESET_TEMP_RST	(0x01 << 0)
#define SIGNAL_PATH_RESET_ACCEL_RST	(0x01 << 1)
#define SIGNAL_PATH_RESET_GYRO_RST	(0x01 << 2)
#define USER_CTRL				0x6A	//106
#define USER_CTRL_I2C_MST_EN	(0x01 << 5)

#define PWR_MGMT_1				0x6B	//107
#define PWR_MGMT_1_CLKSEL_1		(0x01 << 0)
#define PWR_MGMT_1_H_RESET		(0x01 << 7)
#define PWR_MGMT_2				0x6C	//108
#define PWR_MGMT_2_DISABLE_ZG	(0x01 << 0)
#define PWR_MGMT_2_DISABLE_YG	(0x01 << 1)
#define PWR_MGMT_2_DISABLE_XG	(0x01 << 2)
#define PWR_MGMT_2_DISABLE_ZA	(0x01 << 3)
#define PWR_MGMT_2_DISABLE_YA	(0x01 << 4)
#define PWR_MGMT_2_DISABLE_XA	(0x01 << 5)


#define WHO_AM_I				0x75	//117

// ****************** AK8963 MAGNETOMETER REGISTERS *************** //
#define AK8963_WIA				0x00
#define AK8963_HXL				0x03
#define AK8963_HXH				0x04
#define AK8963_HYL				0x05
#define AK8963_HYH				0x06
#define AK8963_HZL				0x07
#define AK8963_HZH				0x08
#define AK8963_ST2				0x09

#define AK8963_CNTL1			0x0A
#define AK8963_CNTL1_16BIT		(0x01 << 4)
#define AK8963_CNTL1_MODE2		(0x06 << 0)
#define AK8963_CNTL1_POWERDOWN	(0x00 << 0)
#define AK8963_CNTL1_FUSEROM	(0x0F << 0)

#define AK8963_CNTL2			0x0B
#define AK8963_CNTL2_SRST		(0x01 << 0)

#define AK8963_ASAX				0x10

#define READ_FLAG   0x80


class mpu9250_mark{
	public:
		mpu9250_mark(
			void (*cs_state_fn) (bool high), 
			unsigned char (*writeSPI1_fn) (uint8_t t),
			void (*delay_ms_fn)(uint32_t ms),
			void (*delay_us_fn)(uint32_t us)
		);
		
		void init();
		uint8_t whoAmI();
		void readAll();
		
		void init_mag();
		uint8_t device_id_mag();
		
		// accel methods ******************
		void read_xyz_accel();
		void read_x_accel();
		void read_y_accel();
		void read_z_accel();
		
		void get_xyz_accel(int16_t *buf);
		int16_t get_x_accel();
		int16_t get_y_accel();
		int16_t get_z_accel();
		
		void get_xyz_accel_mps2(float *buf);
		float get_x_accel_mps2();
		float get_y_accel_mps2();
		float get_z_accel_mps2();
		
		void accel_2g_range();					// +/- 2g range. the 2^16 output from -32768 to 32767 corresponds with -2g to 2g
		void accel_4g_range();
		void accel_8g_range();
		void accel_16g_range();
		
		void update_accel_bias(int16_t x, int16_t y, int16_t z);			// Use raw accel readings to update bias
		void update_x_accel_bias(int16_t xbias);
		void update_y_accel_bias(int16_t ybias);
		void update_z_accel_bias(int16_t zbias);
		
		void update_accel_mult_bias(float x, float y, float z);				// Use raw accel readings to update multiplicative bias
		void update_x_accel_mult_bias(float xbias);
		void update_y_accel_mult_bias(float ybias);
		void update_z_accel_mult_bias(float zbias);
		
		// gyro methods ******************
		void read_xyz_gyro();
		void read_x_gyro();
		void read_y_gyro();
		void read_z_gyro();
		
		void get_xyz_gyro(int16_t *buf);
		int16_t get_x_gyro();
		int16_t get_y_gyro();
		int16_t get_z_gyro();
		
		void get_xyz_gyro_dps(float *buf);
		float get_x_gyro_dps();
		float get_y_gyro_dps();
		float get_z_gyro_dps();
		
		void gyro_250dps_range();
		void gyro_500dps_range();
		void gyro_1000dps_range();
		void gyro_2000dps_range();
		
		void update_gyro_bias(int16_t x, int16_t y, int16_t z);				// these are raw gyro readings (and not _dps readings)
		void update_x_gyro_bias(int16_t xbias);
		void update_y_gyro_bias(int16_t ybias);
		void update_z_gyro_bias(int16_t zbias);
		
		
		// magnetometer methods ******************
		void read_xyz_mag();									// read the current magnetometer values
		float read_x_mag();
		float read_y_mag();
		float read_z_mag();
		
		void get_xyz_mag(float buf[3]);
		float get_x_mag();										// return lastly stored magnetometer value
		float get_y_mag();
		float get_z_mag();
		
		void update_mag_bias(float x, float y, float z);		// update mag bias
		void update_x_mag_bias(float xbias);
		void update_y_mag_bias(float ybias);
		void update_z_mag_bias(float zbias);
		void get_xyz_mag_adj_factor(float adj[3]);
		void set_xyz_mag_adj_factor(float xAdj, float yAdj, float zAdj);
		
		
		// temperature methods ******************
		void read_temperature();
		uint16_t get_raw_temp_out();
		float get_temp();
		
		int8_t init_brian();
		
	//
	private:
		void (*cs_state) (bool);
		unsigned char (*writeSPI1) (uint8_t t);
		void (*delay_ms)(uint32_t ms);
		void (*delay_us)(uint32_t us);
		
		// gyro / accel (mpu9250) read/write functions
		uint8_t writeReg(uint8_t address, uint8_t data);
		uint8_t readReg(uint8_t address);
		void readRegs(uint8_t address, uint8_t *buffer, uint8_t count);
		
		// magnetometer read/write functions
		void writeRegMag(uint8_t address, uint8_t data);
		uint8_t readRegMag(uint8_t address);
		
		int whoAmIAK8963();
		
		
		// Accelerometer ******************
		
		int16_t x_accel;								// accelerometer stuff
		int16_t y_accel;
		int16_t z_accel;
		
		enum accel_range {
			g2,
			g4,
			g8,
			g16
		};
		accel_range accel_cur_range;
		
		int16_t accel_bias[3];							// offset bias (i.e. shift the readings)
		float accel_mult_bias[3];						// multiplicative bias (i.e. stretch/compress the readings)
		
		void update_x_accel(int16_t received);
		void update_y_accel(int16_t received);
		void update_z_accel(int16_t received);
		
		
		// Gyroscope ******************
		
		int16_t x_gyro;									// gyroscope stuff
		int16_t y_gyro;
		int16_t z_gyro;
		
		enum gyro_range {
			dps_250,
			dps_500,
			dps_1000,
			dps_2000
		};
		gyro_range gyro_cur_range;						// the current gyro range
		
		float gyro_bias[3];
		
		void update_x_gyro(int16_t received);
		void update_y_gyro(int16_t received);
		void update_z_gyro(int16_t received);
		
		
		
		// Magnetometer ******************
		
		// magnetometer setup methods
		void get_H_adj();
		void mag_mode_2();
		void setup_mag_readings_automatically();
		
		bool mag_is_setup;
		
		float Hadj_factor[3];							// x, y, z adjustment from fuse rom.
		float x_mag;									// last read magnetometer value
		float y_mag;
		float z_mag;	
		
		float mag_bias[3];								// bias (offset) of magnetometer readings AFTER the multiplicative factor is added
		
		void update_x_mag(int16_t received);			// update the raw magnetometer values into the actual ones
		void update_y_mag(int16_t received);
		void update_z_mag(int16_t received);
		
		uint8_t _buffer[21];
		
		// used to restart the magnetometer. these methods and the init_brian() method successfully restart a hanged-up i2c bus
		int writeAK8963Register(uint8_t subAddress, uint8_t data);
		int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
		int writeRegister(uint8_t subAddress, uint8_t data);
		int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
		
		
		// Temperature ******************
		uint16_t TEMP_OUT;
		float temperature;
		void update_temp();
};

#endif
