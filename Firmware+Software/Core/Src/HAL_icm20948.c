/*
 * icm20948.c
 *
 *  Created on: Jun 20, 2025
 *      Author: jliu3
 */
#include "HAL_icm20948.h"

/*
 * @brief Updates SPI related struct elements so the SPI handle can interface with the IMU
 */
void imu_init_spi(ICM20948 *imu, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *ncs_pin, uint16_t *ncs_pin_bank) {
	// update struct values with corresponding parameters
	imu->spiHandle = spiHandle;
	imu->ncs_pin = ncs_pin;
	imu->ncs_pin_bank = ncs_pin_bank;
}

/*
 * @brief Initializes and configures the IMU to operate in SPI at their maximum sampling rates
 */
void imu_init(imu *imu) {

	// writing a value of 0x00 (00000000) to USER_BANK_SEL register
	// select User Bank 0 to access certain registers for configuration purposes
	write_imu_single(USER_BANK_SEL, USER_BANK_0);

	HAL_Delay(1); // delay used to give some time after user bank change

	// writing a value of 0x80 (10000000) to PWR_MGMT_1 register
	// bit[7] = 1 will reset internal registers on startup
	write_imu_single(PWR_MGMT_1, 0x80);

	HAL_Delay(1); // delay used after reset

	// writing a value of 0x78 (01111000) to the USER_CTRL register
	// bit[6] = 1 enables FIFO operation mode
	// bit[5] = 1 enables the I2C master module
	// bit[4] = 1 resets the I2C slave module, and enables SPI
	// bit[3] = 1 resets the digital motion processing (DMP) module
	write_imu_single(USER_CTRL, 0x78);

	HAL_Delay(1);

	// writing a value of 0x01 (00000001) to the PWR_MGMT_1 register
	// bit[0] to 1 selects the best available clock for the IMU to use
	// in this case, the best clock is the external oscillator the MCU also uses
	write_imu_single(PWR_MGMT_1, 0x01);

	HAL_Delay(1);

	// reset the accelerometer and gyroscope
	// first writes a value of 0x3F (00111111) to the PWR_MGMT_2 register
	// bits[5:0] = 111111 disables all axes on both the accelerometer and gyroscope
	// then writes a value of 0x00 (00000000) to the same register
	// bits[5:0] = 000000 enables all axes on both the accelerometer and gyroscope
	write_imu_single(PWR_MGMT_2, 0x3F);
	HAL_Delay(1); // delay after power off
	write_imu_single(PWR_MGMT_2, 0x00);

	HAL_Delay(1); // delay to give time after power on

	// writing a value of 0x22 (00100010) to the INT_PIN_CFG register
	// bit[7] = 0 sets logic level for interrupt as high
	// bit[6] = 0 configures the interrupt pin as push-pull
	// bit[5] = 1 sets the interrupt pin as high until the interrupt status is cleared
	// bit[4] = 0 means the interrupt status is only cleared by reading INT_STATUS
	// bit[1] = 1 sets the I2C master interface pins into "bypass mode" when the I2C master interface is disabled
	write_imu_single(INT_PIN_CFG, 0x22);

	// writing a value of 0x01 (00000001) to the INT_ENABLE_1 register
	// bit[0] = 1 enables raw data ready interrupt to propagate to interrupt pin 1
	write_imu_single(INT_ENABLE_1, 0x01);

	// writing a value of 0x02 (00000010) to the USER_BANK_SEL register
	// select User Bank 2 to access certain registers for configuration purposes
	write_imu_single(USER_BANK_SEL, USER_BANK_2);

	HAL_Delay(1); // delay used to give some time after user bank change

	// writing a value of 0x01 (00000001) to the ODR_ALIGN_EN register
	// bit[0] = 1 enables output data rate (ODR) starting alignment
	// this means that the start times of different sensors are synchronized
	write_imu_single(ODR_ALIGN_EN, 0x01);

	// writing a value of 0x13 (00010011) to the GYRO_CONFIG_1 register
	// bit[5:3] = 010 selects a low pass filter configuration where the 3DB Bandwidth is 119.5 Hz
	// bit[2:1] = 01 selects a resolution scale of +/- 500 DPS
	// bit[0] = 1 enables the digital low pass filter (DLPF) for the gyroscope
	write_imu_single(GYRO_CFG_1, 0x13);

	// writing a value of 0x00 (00000000) to the GYRO_SMPLRT_DIV register
	// ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
	// bit[7:0] = 00000000 sets the gyroscope sampling rate to its full 1.1 kHz
	write_imu_single(GYRO_SMPL_RATE, 0x00);

	// writing a value of 0x13 (00010011) to the ACCEL_CONFIG_1 register
	// bit[5:3] = 010 selects a low pass filter configuration where the 3DB Bandwidth is 111.4 Hz
	// bit[2:1] = 01 selects a resolution scale of +/- 4g
	// bit[0] = 1 enables the digital low pass filter (DLPF) for the accelerometer
	write_imu_single(ACCEL_CFG_1, 0x13);

	// writing a value of 0x00 (00000000) both ACCEL_SMPLRT_DIV registers
	// ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
	// bits[11:0] = 000000000000 sets the accelerometer sampling rate to its full 1.125 kHz
	write_imu_single(ACCEL_SMPL_RATE_1, 0x00);
	write_imu_single(ACCEL_SMPL_RATE_2, 0x00);
}

/*
 * @brief Initializes and configures the AK09166 to operate in I2C peripheral mode at its maximum sampling rate
 */
void mag_init(void) {

	// writing a value of 0x00 (00000000) to USER_BANK_SEL register
	// select user bank 0 to configure the I2C master module
	write_imu_single(USER_BANK_SEL, USER_BANK_0);

	// editing the register value at USER_CTRL to include 0x02 (00000010), using the "or" bitwise operator
	// bit[2] = 1 triggers an I2C master module reset
	uint8_t register_edit; // temporary variable used to edit an existing register value
	read_imu_single(USER_CTLR, &register_edit); // get current register value
	register_edit |= 0x02; // edit register value to reset I2C master module
	write_imu_single(USER_CTRL, register_edit); // writes new value to the register

	HAL_Delay(1); // delay after the reset

	// edit the register value at USER_CTLR to include 0x2 (00100000), using the "or" bitwise operator
	// bit[5] = 1 re-enables the I2C master module, which we use to interface with the AK09916
	register_edit |= 0x20;
	write_imu_single(USER_CTLR, register_edit);

	// write_imu_single(INT_PIN_CFG, 0x30); // configuring interrupts

	// writing a value of 0x00 (00000000) to the LP_CFG register
	// bit[6:4] = 000 disables duty cycled mode for the I2C master module (and other sensors)
	// this allows for continuous polling
	write_imu_single(LP_CFG, 0x00);

	// writing a value of 0x00 (00000011) to USER_BANK_SEL register
	// select user bank 3 to configure the I2C master clock frequency
	write_imu_single(USER_BANK_SEL, USER_BANK_3);

	// also write a value of 0x07 (00000111) to the I2C_MST_CTLR register
	// bit[3:0] = 1111 sets the I2C master module frequency to 400 kHz
	write_imu_single(I2C_MST_CTRL, 0x07);

	// writes a value of 0x01 (00000001) to AK09916 register CNTL3
	// bit[0] = 1 triggers a reset
	write_mag_single(MAG_CTRL_3, 0x01);
	HAL_Delay(1); // delay after reset
	// writes a value of 0x08 (00001000) to AK09916 register CNTL2
	// bit[4:0] = 01000 sets the AK09916 to continuous measurement mode 4 (100Hz)
	write_mag_single(MAG_CTRL_2, 0x08);

	// begin reading AK09916 data
	// once read once, I2C will automatically sample data
	uint8_t temp; // temporary variable to satisfy parameter requirements
	read_mag_single(EXT_SLV_SENS_DATA_00, &temp);

	// writing a value of 0x00 (00000000) to USER_BANK_SEL register
	// this selects user bank 0 so that accelerometer, gyroscope, and AK09916 data can be accessed
	write_imu_single(USER_BANK_SEL, USER_BANK_0);

}

/*
 * @brief Write data to a specified IMU register
*  @params reg_addr: the desired ICM20948 register address to write to
 * @params TxData: data to be written to the register
 * @return Status of the data transfer
 */
uint8_t write_imu_single(ICM20948 *imu, uint8_t reg_addr, uint8_t TxData) {
	// initialize TxBuffer consisting of the register address and data to be written
	uint8_t TxBuffer[2] = {reg_addr, TxData};

	HAL_GPIOWritePin(imu->ncs_pin_bank, imu->ncs_pin, GPIO_PIN_RESET); // pull NCS low for the IMU, selecting it for data transfer
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, TxBuffer, sizeof(TxBuffer), 1) == HAL_OK); // write data to specified register
	HAL_GPIOWritePin(imu->ncs_pin_bank, imu->ncs_pin, GPIO_PIN_SET); // pull NCS high for IMU, de-selecting it for data transfer

	return status; // return the status of the data transfer
}

/*
 * @brief Write data to a specified IMU register
*  @params reg_addr: the desired ICM20948 register address to write to
 * @params TxData: data to be written to the register
 * @params size: size of the data buffer to be transmitted
 * @return Status of the data transfer
 */
uint8_t write_imu_multiple(ICM20948 *imu, uint8_t reg_addr, uint8_t *TxData, uint8_t size) {
	uint8_t TxBuffer[size+1]; // initialize TxBuffer
	TxBuffer[0] = reg_addr; // set first index as register address
	// loop through each byte in TxData to add to the transmit buffer TxBuffer
	for (uint8_t i = 0; i < size; i++) {
		TxBuffer[i+1] = TxData[i];
	}

	HAL_GPIOWritePin(imu->ncs_pin_bank, imu->ncs_pin, GPIO_PIN_RESET); // pull NCS low for the IMU, selecting it for data transfer
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, TxBuffer, size+1, 1) == HAL_OK); // burst write data to specified register if HAL status is OK
	HAL_GPIOWritePin(imu->ncs_pin_bank, imu->ncs_pin, GPIO_PIN_SET); // pull NCS high for IMU, de-selecting it for data transfer

	return status; // return the status of the data transfer
}

/*
 * @brief Read a single byte of data from a specified IMU register
 * @params the desired ICM20948 register address to read from
 * @params data: a pointer to the data buffer the read data is stored at
 * @return Status of the data transfer
 */
uint8_t read_imu_single(ICM20948 *imu, uint8_t reg_addr, uint8_t *RxData) {
	// initialize TxData buffer consisting of the register address combined with 0x80 (10000000)
	// bit[7] = 1 signifies a read operation
	// initialize RxData buffer to receive data read from the specified register
	uint8_t TxBuffer = (reg_addr | 0x80);

	HAL_GPIOWritePin(imu->ncs_pin_bank, imu->ncs_pin, GPIO_PIN_RESET); // pull NCS low for the IMU, selecting it for data transfer
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, &TxBuffer, RxData, 1, 1) == HAL_OK); // simultaneous write/read operation on the specified register
	HAL_GPIOWritePin(imu->ncs_pin_bank, imu->ncs_pin, GPIO_PIN_SET); // pull NCS high for IMU, de-selecting it for data transfer

	return status; // return status of the data transfer
}

/*
 * @brief Read multiple bytes of data from a specified IMU register
 * @params the desired ICM20948 register address to read from
 * @params data: a pointer to the data buffer the read data is stored at
 * @params size: size of the data buffer to be transmitted
 * @return Status of the data transfer
 */
uint8_t read_imu_multiple(ICM20948 *imu, uint8_t reg_addr, uint8_t *RxData, uint8_t size) {
	// initialize TxData buffer consisting of the register address combined with 0x80 (10000000)
	// bit[7] = 1 signifies a read operation
	// initialize RxData buffer to receive data read from the specified register
	uint8_t TxBuffer = (reg_addr | 0x80);

	HAL_GPIOWritePin(imu->ncs_pin_bank, imu->ncs_pin, GPIO_PIN_RESET); // pull NCS low for the IMU, selecting it for data transfer
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, &TxBuffer, RxData, size, 1) == HAL_OK); // simultaneous write/read operation on the specified register
	HAL_GPIOWritePin(imu->ncs_pin_bank, imu->ncs_pin, GPIO_PIN_SET); // pull NCS high for IMU, de-selecting it for data transfer

	return status; // return status of the data transfer
}

/*
 * @brief Write data to a specified AK09916 register
 * @params reg_addr: the desired AK09166 register address to write to
 * @params TxData: data to be written to the register
 */
void write_mag_single(uint8_t reg_addr, uint8_t TxData) {
	// writing a value of 0x00 (00000011) to USER_BANK_SEL register
	// select User Bank 3 to access AK09166 registers
	write_imu_single(USER_BANK_SEL, USER_BANK_3);
	// writing the peripheral address (0x0C) of the AK09166 to the I2C_SLV0_ADDR register
	// bit[7] = 0 indicates a write operation
	// this defines the AK09166 as an I2C peripheral for write operations
	write_imu_single(I2C_SLV0_ADDR, 0x0C);
	// writing the desired register address within the AK09166 to begin data transmission with
	write_imu_single(I2C_SLV0_REG, reg_addr);
	// data is written to I2C_SVL0_D0, where it gets written to the specified register within the AK09166
	write_imu_single(I2C_SLV0_DO, TxData);
}

/*
 * @brief Read data from a specified AK09916 register
 * @params reg_addr: the desired AK09166 register address to read from
 * @params RxBuffer: a pointer to the data buffer the read data is stored at
 */
void read_mag_single(uint8_t reg_addr, uint8_t *RxBuffer) {
	// writing a value of 0x00 (00000011) to USER_BANK_SEL register
	// select User Bank 3 to access AK09166 registers
	write_imu_single(USER_BANK_SEL, USER_BANK_3);
	// writing the peripheral address (0x0C) of the AK09166 to the I2C_SLV0_ADDR register
	// use bitwise "or" operator to include 0x80 (10000000), bit[7] = 1 indicates a read operation
	// this defines the AK09166 as an I2C peripheral for read operations
	write_imu_single(I2C_SLV0_ADDR, (0x0C | 0x80));
	// writing the desired register address within the AK09166 to begin data transmission with
	write_imu_single(I2C_SLV0_REG, reg_addr);

	write_imu_single(I2C_SLV0_DO, 0xFF); // read

	// writing a value of 0x00 (00000011) to USER_BANK_SEL register
	// select User Bank 0 to access data out registers for the AK09916
	write_imu_single(USER_BANK_SEL, USER_BANK_0);

	// AK09916 data output gets put to EXT_SLV_SENS_DATA_00
	read_imu_single(EXT_SLV_SENS_DATA_00, RxBuffer);
}


/*
 * @brief Reads WHO_AM_I register, used to check hardware functionality before configuration.
 * @return Nominally should be an integer with value 0xEA
 */
uint8_t who_am_i(void) {
	uint8_t imu_id; // initialize variable to store byte
	// reading from the WHO_AM_I register
	// stores read data into imu_id
	write_imu_single(USER_BANK_SEL, USER_BANK_0);
	read_imu_single(WHO_AM_I, &imu_id);
	// imu_id should be 0xEA
	return imu_id;
}

/*
 * @brief Read accelerometer XYZ components and assign them to corresponding IMU struct data elements
 * @params imu: A pointer towards the IMU struct, which contains relevant data structures
 * @return Status of the data transfer
 */
uint8_t read_accel_vec(imu *imu) {

	// initialize Rx buffer for burst read
	uint8_t RxBuffer[6];

	// set up dummy bytes for burst read
	//for (uint8_t i = 1; i < 7; i++) TxData[i] = 0xFF;

	// burst read to fill RxBuffer
	// note: AK09166 uses big-endian (MSB first) in its register map
	// hence why we shift the MSB left by 8 bits
	// the bitwise "or" operator is used to combine the MSB and LSB to create a 16 bit integer
	uint8_t status = read_imu_multiple(0x2D, RxBuffer, 6);

	// only update is SPI transaction is successful
	if (status == 0) {
		// assign raw values to corresponding vector components
		imu->A_raw[0] = (RxBuffer[0] << 8) | RxBuffer[1];
		imu->A_raw[1] = (RxBuffer[2] << 8) | RxBuffer[3];
		imu->A_raw[2] = (RxBuffer[4] << 8) | RxBuffer[5];

		// convert them to proper measurements
		imu->A[0] = imu->accel_conversion * imu->A_raw[0];
		imu->A[1] = imu->accel_conversion * imu->A_raw[1];
		imu->A[2] = imu->accel_conversion * imu->A_raw[2];
	}

	return status;
}

/*
 * @brief Read gyroscope XYZ components and assign them to corresponding IMU struct data elements
 * @params imu: A pointer towards the IMU struct, which contains relevant data structures
 * @return Status of the data transfer
 */
uint8_t read_gyro_vec(imu *imu) {

	// initialize Tx/Rx buffers for burst read
	uint8_t TxData;
	uint8_t RxBuffer[6];

	// set up dummy bytes for burst read
	//for (uint8_t i = 1; i < 7; i++) TxData[i] = 0xFF;

	// burst read to fill RxBuffer
	// note: AK09166 uses big-endian (MSB first) in its register map
	// hence why we shift the MSB left by 8 bits
	// the bitwise "or" operator is used to combine the MSB and LSB to create a 16 bit integer
	uint8_t status = read_imu_multiple(0x31, RxBuffer, 6);

	// only update is SPI transaction is successful
	if (status == 0) {
		// assign raw values to corresponding vector components
		imu->G_raw[0] = (RxBuffer[0] << 8) | RxBuffer[1];
		imu->G_raw[1] = (RxBuffer[2] << 8) | RxBuffer[3];
		imu->G_raw[2] = (RxBuffer[4] << 8) | RxBuffer[5];

		// convert them to proper measurements
		imu->G[0] = imu->gyro_conversion * imu->G_raw[0];
		imu->G[1] = imu->gyro_conversion * imu->G_raw[1];
		imu->G[2] = imu->gyro_conversion * imu->G_raw[2];
	}

	return status;

}

/*
 * @brief Read AK09916 XYZ components and assign them to corresponding IMU struct data elements
 * @params imu: A pointer towards the IMU struct, which contains relevant data structures
 */
void read_mag_vec(imu *imu) {

	// initialize 6 byte Rx buffer to store raw data
	uint8_t RxBuffer[6];

	// initialize burst read starting from the EXT_SLV_SENS_DATA_00 register
	// AK09166 measurements should be stored between EXT_SLV_SENS_DATA_00 and EXT_SLV_SENS_DATA_05
	uint8_t status = read_imu_multiple(EXT_SLV_SENS_DATA_00, RxBuffer, 6);

	// only update is SPI transaction is successful
	if (status == 0) {
		// assign raw values to corresponding vector components
		// note: AK09166 uses little-endian (LSB first) in its register map
		// this is why combining bytes seems reversed compared to the accelerometer or gyroscope
		imu->M_raw[0] = (RxBuffer[1] << 8) | RxBuffer[0];
		imu->M_raw[1] = (RxBuffer[3] << 8) | RxBuffer[2];
		imu->M_raw[2] = (RxBuffer[5] << 8) | RxBuffer[4];

		// convert them to proper measurements
		// according to the data sheet, typical sensitivity/conversion rate should be 0.15 μT/LSB
		imu->M[0] = imu->mag_conversion * imu->M_raw[0];
		imu->M[1] = imu->mag_conversion * imu->M_raw[1];
		imu->M[2] = imu->mag_conversion * imu->M_raw[2];
	}

	read_mag_single(0x18); // reading register STATUS_2 is required as end register to stop reading

}

/*
 * @brief Calibrate the gyroscope by averaging its measurement bias and writing those values to their corresponding offset registers
 * @params imu: A pointer towards the IMU struct, which contains relevant data structures
 */
void calibrate_gyro(imu *imu) {
	int32_t Gx_bias, Gy_bias, Gz_bias; // int32_t to avoid overflow
	uint8_t sample_size = 50; // collect 50 gyroscope measurements to average
	int16_t gyro_offset[6]; // array containing 2 byte pairs of LSB and MSB for gyroscope bias

	// loop that accumulates the specified amount of raw data samples
	for(uint8_t i = 0; i <= sample_size; i++) {
		read_gyro_vec(imu);
		Gx_bias += imu->G_raw[0];
		Gy_bias += imu->G_raw[1];
		Gz_bias += imu->G_raw[2];
		HAL_delay(1); // 1ms delay as polling rate is >1kHz
	}

	// averaging out
	Gx_bias /= sample_size;
	Gy_bias /= sample_size;
	Gz_bias /= sample_size;

	// dividing by 4 is needed to get 32.9 LSB per deg/s to conform to expected bias input format
	// change bias value to negative as we want to subtract the measurement error
	Gx_bias /= -4;
	Gy_bias /= -4;
	Gz_bias /= -4;

	// format as 2 byte values
	// each bias value is combined with 0xFF via an "and" operation to preserve correct values
	gyro_offset[0] = ((Gx_bias << 8) & 0xFF);
	gyro_offset[1] = (Gx_bias & 0xFF);
	gyro_offset[2] = ((Gy_bias << 8) & 0xFF);
	gyro_offset[3] = (Gy_bias & 0xFF);
	gyro_offset[4] = ((Gz_bias << 8) & 0xFF);
	gyro_offset[5] = (Gz_bias & 0xFF);

	// burst write offsets to corresponding registers
	// we can use burst write here as the registers are structured in order
	// these registers store offset values and automatically use them to compensate for bias during measurements
	write_imu_multiple(XG_OFFS_USRH, gyro_offset, 6, 1);
}

/*
 * @brief Calibrate the gyroscope by averaging its measurement bias and writing those values to their corresponding offset registers
 * @params imu: A pointer towards the IMU struct, which contains relevant data structures
 */
void calibrate_accel(imu *imu) {
	int32_t Ax_bias, Ay_bias, Az_bias; // int32_t to avoid overflow
	uint8_t sample_size = 50; // collect 50 gyroscope measurements to average
	int16_t accel_offset[6]; // array containing 2 byte pairs of LSB and MSB for gyroscope bias

	// loop that accumulates the specified amount of raw data samples
	for(uint8_t i = 0; i <= sample_size; i++) {
		read_gyro_vec(imu);
		Ax_bias += imu->A_raw[0];
		Ay_bias += imu->A_raw[1];
		Az_bias += imu->A_raw[2];
		HAL_delay(1); // 1ms delay as polling rate is >1kHz
	}

	// averaging out
	Ax_bias /= sample_size;
	Ay_bias /= sample_size;
	Az_bias /= sample_size;

	// dividing by 4 is needed to get 32.9 LSB per deg/s to conform to expected bias input format
	// change bias value to negative as we want to subtract the measurement error
	Ax_bias /= -4;
	Ay_bias /= -4;
	Az_bias /= -4;

	// format as 2 byte values
	// each bias value is combined with 0xFF via an "and" operation to preserve correct values
	accel_offset[0] = ((Ax_bias << 8) & 0xFF);
	accel_offset[1] = (Ax_bias & 0xFF);
	accel_offset[2] = ((Ay_bias << 8) & 0xFF);
	accel_offset[3] = (Ay_bias & 0xFF);
	accel_offset[4] = ((Az_bias << 8) & 0xFF);
	accel_offset[5] = (Az_bias & 0xFF);

	// write offsets to corresponding registers
	// cannot use burst read due to the register structure not being ordered
	// these registers store offset values and automatically use them to compensate for bias during measurements
	write_imu_single(XA_OFFS_USRH, accel_offset[0]);
	write_imu_single(XA_OFFS_USRL, accel_offset[1]);
	write_imu_single(YA_OFFS_USRH, accel_offset[2]);
	write_imu_single(YA_OFFS_USRL, accel_offset[3]);
	write_imu_single(ZA_OFFS_USRH, accel_offset[4]);
	write_imu_single(ZA_OFFS_USRL, accel_offset[5]);
}

// might not create a calibrate compass function as precise heading information is not necessary at this phase in the project
void calibrate_mag(void) {

}
