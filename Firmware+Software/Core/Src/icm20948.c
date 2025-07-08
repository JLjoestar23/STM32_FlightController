/*
 * icm20948.c
 *
 *  Created on: Jun 20, 2025
 *      Author: jliu3
 */
#include "icm20948.h"


uint8_t imu_init(imu *imu) {

	write_imu_reg(PWR_MGMT_1, 0x80); // reset internal registers on startup

	HAL_Delay(1);

	write_imu_reg(USER_BANK_SEL, USER_BANK_0); // select User Bank 0 for configuration

	HAL_Delay(1);

	// WHO AM I
	uint8_t imu_id = 0xFF;
	read_imu_reg(WHO_AM_I, &imu_d);

	write_imu_reg(USER_CTLR, 0x78); // disable I2C, which enables SPI

	HAL_Delay(1);

	write_imu_reg(PWR_MGMT_1, 0x01); // select best available clock

	HAL_Delay(1);

	// reset accelerometer and gyroscope
	write_imu_reg(PWR_MGMT_2, 0x3F);
	HAL_Delay(1);
	write_imu_reg(PWR_MGMT_2, 0x00);

	HAL_Delay(1);

	write_imu_reg(USER_BANK_SEL, USER_BANK_2); // select User Bank 2 for configuration

	HAL_Delay(1);

	write_imu_reg(ODR_ALIGN_EN, 0x01); // enables output data rate (ODR) starting alignment

	write_imu_reg(GYRO_CFG_1, 0x13); // configure digital low pass filter and +/- 500 DPS range for gyroscope

	write_imu_reg(GYRO_SMPL_RATE, 0x00); // set gyroscope sampling rate to its full 1.1kHz

	write_imu_reg(ACCEL_CFG_1, 0x13); // configure digital low pass filter and +/- 4g range for accelerometer

	// set accelerometer sampling rate to 1.125kHz
	write_imu_reg(ACCEL_SMPL_RATE_1, 0x00);
	write_imu_reg(ACCEL_SMPL_RATE_2, 0x00);

	mag_init(); // initialize magnetometer

	read_mag_reg(BEGIN_VEC_DATA, 8); // begin reading magnetometer data, once read once, I2C will automatically sample data

	write_imu_reg(USER_BANK_SEL, USER_BANK_0); // select User Bank 0 to get readings

}

void mag_init() {

	write_imu_reg(USER_BANK_SEL, USER_BANK_0); // Select User Bank 0

	uint8_t register_edit; // temporary variable used to edit existing register values
	read_imu_reg(USER_CTLR, &register_edit); // get current register value
	register_edit |= 0x02; // edit register value to reset I2C master module
	write_imu_reg(USER_CTRL, register_edit);

	HAL_Delay(1);

	register_edit |= 0x20; // edit register value to enable I2C master module
	write_imu_reg(USER_CTLR, register_edit);

	// write_imu_reg(INT_PIN_CFG, 0x30); // configuring interrupts

	write_imu_reg(USER_BANK_SEL, USER_BANK_3); // Select User Bank 3
	write_imu_reg(I2C_MST_CTRL, 0x07); // I2C master clock set to 400kHz

	write_imu_reg(USER_BANK_SEL, USER_BANK_0); // Select User Bank 0
	write_imu_reg(LP_CFG, 0x00); // operate I2C master in continuous polling mode

	write_mag_reg(MAG_CTRL3, 0x01); // magnetometer reset
	HAL_Delay(1);
	write_mag_reg(MAG_CTRL2, 0x02); // set magnetometer to continuous measurement mode 4 (100Hz)

}


uint8_t write_imu_reg(uint8_t reg_addr, uint8_t data) {

	uint8_t TxData[2] = {reg_addr, data};

	peripheral_select(ICM20948_NCS);
	uint8_t status = SPI_transmit(*TxData, sizeof(TxData), 5);
	peripheral_deselect(ICM20948_NCS);

	return status;

}

uint8_t read_imu_reg(uint8_t reg_addr, uint8_t *data) {

	write_imu_reg(USER_BANK_SEL, USER_BANK_0); // Select User Bank 0

	uint8_t TxData[3] = {(reg_addr | 0x80), 0x00, 0x00};
	uint8_t RxData[3];

	peripheral_select(ICM20948_NCS);
	uint8_t status = SPI_transmit_receive(*TxData, *RxData, sizeof(TxData), 5);
	peripheral_deselect(ICM20948_NCS);

	if (status == 1) {

		*data = RxData[2];

	}

	return status;

}

void write_mag_reg(uint8_t reg_addr, uint8_t data) {
	write_imu_reg(USER_BANK_SEL, USER_BANK_3); // Select User Bank 3
	write_imu_reg(I2C_SLV0_ADDR, 0x0C); // define magnetometer as I2C peripheral for write operation
	write_imu_reg(I2C_SLV0_REG, reg_addr); // define register address (within magnetometer) to begin data transfer with
	write_imu_reg(I2C_SLV0_DO, data); // set data to be written
}

uint8_t read_mag_reg(uint8_t reg_addr) {

	uint8_t data;
	write_imu_reg(USER_BANK_SEL, USER_BANK_3); // Select User Bank 3
	write_imu_reg(I2C_SLV0_ADDR, (0x0C | 0x80)); // define magnetometer as I2C peripheral for read operation
	write_imu_reg(I2C_SLV0_REG, reg_addr); // define register address (within magnetometer) to begin data transfer with
	write_imu_reg(I2C_SLV0_D0, 0xFF); // read
	write_imu_reg(USER_BANK_SEL, USER_BANK_0); // Select User Bank 0
	read_imu_reg(MAG_DATA_OUT_1, &data);
	return data;
}

/*
 * @brief
 */
uint8_t read_accel_vec(imu *imu) {

	// initialize Tx/Rx buffers for burst read
	uint8_t TxData[7];
	uint8_t RxData[7];

	// Tx buffer to be sent to 0x2D, which is the register addr of ACCEL_X
	TxData[0] = (0x2D | 0x80);

	// set up dummy bytes for burst read
	for (uint8_t i = 1; i < 7; i++) TxData[i] = 0xFF;

	write_imu_reg(USER_BANK_SEL, USER_BANK_0); // Select User Bank 0
	// transmit-receive process using burst read
	peripheral_select(ICM20948_NCS);
	uint8_t status = SPI_transmit_receive(*TxData, *RxData, sizeof(TxData), 5);
	peripheral_deselect(ICM20948_NCS);

	if (status == 1) {
		// assign raw values to corresponding vector components
		int16_t Ax = (RxData[1] << 8) | RxData[2];
		int16_t Ay = (RxData[3] << 8) | RxData[4];
		int16_t Az = (RxData[5] << 8) | RxData[6];

		// convert them to proper measurements
		imu->A[0] = imu->accel_conversion * Ax;
		imu->A[1] = imu->accel_conversion * Ay;
		imu->A[2] = imu->accel_conversion * Az;
	}

	return status;

}

uint8_t read_gyro_vec(imu *imu) {

	// initialize Tx/Rx buffers for burst read
	uint8_t TxData[7];
	uint8_t RxData[7];

	// Tx buffer to be sent to 0x2D, which is the register address of ACCEL_X
	TxData[0] = (0x31 | 0x80);

	// set up dummy bytes for burst read
	for (uint8_t i = 1; i < 7; i++) TxData[i] = 0xFF;

	write_imu_reg(USER_BANK_SEL, USER_BANK_0); // Select User Bank 0
	// transmit-receive process
	peripheral_select(ICM20948_NCS);
	uint8_t status = SPI_transmit_receive(*TxData, *RxData, sizeof(TxData), 5);
	peripheral_deselect(ICM20948_NCS);

	if (status == 1) {
		// assign raw values to corresponding vector components
		int16_t Gx = (RxData[1] << 8) | RxData[2];
		int16_t Gy = (RxData[3] << 8) | RxData[4];
		int16_t Gz = (RxData[5] << 8) | RxData[6];

		// convert them to proper measurements
		imu->G[0] = imu->accel_conversion * Gx;
		imu->G[1] = imu->accel_conversion * Gy;
		imu->G[2] = imu->accel_conversion * Gz;
	}

	return status;

}

void read_mag_vec(imu *imu) {

	uint8_t mag_buffer[6];

	mag_buffer[0] = read_mag_reg(0x01);
	mag_buffer[1] = read_mag_reg(0x11);
	mag_buffer[2] = read_mag_reg(0x12);

	int16_t Mx = mag_buffer[1] | (mag_buffer[2]<<8);

	mag_buffer[3] = read_mag_reg(0x13);
	mag_buffer[4] = read_mag_reg(0x14);

	int16_t My = mag_buffer[3] | (mag_buffer[4]<<8);

	mag_buffer[5] = read_mag_reg(0x15);
	mag_buffer[6] = read_mag_reg(0x16);

	int16_t Mz = mag_buffer[5] | (mag_buffer[6]<<8);

	read_mag_reg(0x18); // reading register STATUS_2 is required as end register to stop reading

	imu->M[0] = imu->mag_conversion * Mx;
	imu->M[1] = imu->mag_conversion * My;
	imu->M[2] = imu->mag_conversion * Mz;

}
