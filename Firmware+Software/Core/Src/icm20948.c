/*
 * icm20948.c
 *
 *  Created on: Jun 20, 2025
 *      Author: jliu3
 */
#include "icm20948.h"


uint8_t imu_init(imu *imu) {

	// select User Bank 0 for configuration
	write_imu_reg(USER_BANK_SEL, USER_BANK_0);

	HAL_Delay(1);

	// get
	uint8_t imu_id = 0xFF;
	read_imu_reg(WHO_AM_I, &imu_d);

	// disable I2C, which enables SPI
	write_imu_reg(USER_CTLR, 0x78);

	HAL_Delay(1);

	// select best available clock
	write_imu_reg(PWR_MGMT_1, 0x01);

	HAL_Delay(1);

	// reset accel + gyro
	write_imu_reg(PWR_MGMT_2, 0x3F);
	HAL_Delay(1);
	write_imu_reg(PWR_MGMT_2, 0x00);

	HAL_Delay(1);

	// select User Bank 2 for configuration
	write_imu_reg(USER_BANK_SEL, USER_BANK_2);

	HAL_Delay(1);

	// configure digital low pass filter and +/- 500 DPS range for gyro
	write_imu_reg(GYRO_CFG_1, 0x13);

	// set gyro sampling rate to 550Hz
	write_imu_reg(GYRO_SMPL_RATE, 0x01);

	// configure digital low pass filter and +/- 4g range for acccelerometer
	write_imu_reg(ACCEL_CFG_1, 0x13);

	// set accelerometer sampling rate to 562.5 Hz
	write_imu_reg(ACCEL_SMPL_RATE_1, 0x00);
	write_imu_reg(ACCEL_SMPL_RATE_2, 0x01);

	// configure AUX_I2C for the onboard magnetometer
	write_imu_reg(USER_BANK_SEL, USER_BANK_0); // Select User Bank 0
	write_imu_reg(INT_PIN_CFG, 0x30); // configuring interrupts
	// write_imu_reg(USER_CTRL, 0x20); // I2C_MST_EN already enabled previously
	write_imu_reg(USER_BANK_SEL, USER_BANK_3); // Select User Bank 3
	write_imu_reg(I2C_MST_CTRL, 0x4F); // I2C Master mode and Speed 400 kHz
	write_imu_reg(I2C_MST_DELAY_CTRL, 0x01); // I2C_SLV0_DLY_EN
	write_imu_reg(0x05, 0x81); // enable IIC	and EXT_SENS_DATA==1 Byt

}


uint8_t write_imu_reg(uint8_t reg_addr, uint8_t data) {

	float TxData[2] = {reg_addr, data};

	peripheral_select(ICM20948_NCS);
	uint8_t status = SPI_write(*TxData, sizeof(TxData), 5);
	peripheral_deselect(ICM20948_NCS);

	return status;

}

uint8_t read_imu_reg(uint8_t reg_addr, uint8_t *data) {

	float TxData[3] = {(reg_addr | 0x80), 0x00, 0x00};
	float RxData[3];

	peripheral_select(ICM20948_NCS);
	uint8_t status = SPI_read_write(*TxData, *RxData, sizeof(TxData), 5);
	peripheral_deselect(ICM20948_NCS);

	if (status == 1) {

		*data = RxData[2];

	}

	return status;

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

	// transmit-receive process
	peripheral_select(ICM20948_NCS);
	uint8_t status = SPI_read_write(*TxData, *RxData, sizeof(TxData), 5);
	peripheral_deselect(ICM20948_NCS);

	if (status == 1) {
		// assign raw values to corresponding vector components
		int16_t Ax = (RxData[1] << 8) | RxData[2];
		int16_t Ay = (RxData[3] << 8) | RxData[4];
		int16_t Az = (RxData[5] << 8) | RxData[8];

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

	// Tx buffer to be sent to 0x2D, which is the register addr of ACCEL_X
	TxData[0] = (0x31 | 0x80);

	// set up dummy bytes for burst read
	for (uint8_t i = 1; i < 7; i++) TxData[i] = 0xFF;

	// transmit-receive process
	peripheral_select(ICM20948_NCS);
	uint8_t status = SPI_read_write(*TxData, *RxData, sizeof(TxData), 5);
	peripheral_deselect(ICM20948_NCS);

	if (status == 1) {
		// assign raw values to corresponding vector components
		int16_t Gx = (RxData[1] << 8) | RxData[2];
		int16_t Gy = (RxData[3] << 8) | RxData[4];
		int16_t Gz = (RxData[5] << 8) | RxData[8];

		// convert them to proper measurements
		imu->G[0] = imu->accel_conversion * Gx;
		imu->G[1] = imu->accel_conversion * Gy;
		imu->G[2] = imu->accel_conversion * Gz;
	}

	return status;

}
