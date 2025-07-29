/*
 * icm20948.h
 *
 *  Created on: Jun 17, 2025
 *      Author: jliu3
 */

#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_

#include "stdint.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#define IMU_ACCEL_READ 			0x2D

// user bank selection related
#define USER_BANK_SEL			0x7F
#define USER_BANK_0 			0x00
#define USER_BANK_1				0x10
#define USER_BANK_2				0x20
#define USER_BANK_3				0x30

// configuration related registers
// user bank 0
#define WHO_AM_I				0x00
#define USER_CTRL				0x03
#define LP_CFG					0x05
#define PWR_MGMT_1				0x06
#define PWR_MGMT_2				0x07
#define INT_PIN_CFG				0x0F
#define INT_ENABLE_1			0x17
#define MAG_DATA_OUT_1			0x3B
#define FIFO_EN_1				0x66
#define FIFO_EN_2				0x67
#define FIFO_RST				0x68
#define FIFO_MODE				0x69
#define FIFO_COUNTH				0x70
#define FIFO_R_W				0x72

// user bank 1
#define XA_OFFS_USRH			0x14
#define XA_OFFS_USRL			0x15
#define YA_OFFS_USRH			0x17
#define YA_OFFS_USRL			0x18
#define ZA_OFFS_USRH			0x1A
#define ZA_OFFS_USRL			0x1B

// user bank 2
#define GYRO_SMPL_RATE			0x00
#define GYRO_CFG_1				0x01
#define XG_OFFS_USRH			0x03
#define XG_OFFS_USRL			0x04
#define YG_OFFS_USRH			0x05
#define YG_OFFS_USRL			0x06
#define ZG_OFFS_USRH			0x07
#define ZG_OFFS_USRL			0x08
#define ODR_ALIGN_EN			0x09
#define ACC_SMPL_RATE_1			0x10
#define ACC_SMPL_RATE_2			0x11
#define ACC_CFG_1				0x14


// user bank 3
#define I2C_MST_CTRL			0x01
#define I2C_MST_DELAY_CTRL		0x02
#define I2C_SLV0_ADDR			0x03
#define I2C_SLV0_REG			0x04
#define I2C_SLV0_CTRL			0x05
#define I2C_SLV0_DO				0x06

// AK09916 registers
#define EXT_SLV_SENS_DATA_00	0x11
#define MAG_CTRL_2				0x31
#define MAG_CTRL_3				0x32

// struct consisting of IMU related variables
typedef struct {

	// SPI related elements
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef *ncs_pin;
	uint16_t *ncs_pin_bank;

	// raw x, y, z component measurements
	int32_t A_raw[3];
	int32_t G_raw[3];
	int32_t M_raw[3];

	// x, y, z component measurements
	float A[3]; // accelerometer vector
	float G[3]; // gyroscope vector
	float M[3]; // AK09916 vector

	// conversion constants to be applied to raw measurements
	float accel_conversion = 1/8192; // sensitivity scale factor is 8192 LSB/g for a resolution of +/- 4g
	float gyro_conversion = 1/65.5; // sensitivity scale factor is 65.5 LSB/dps for a resolution of +/- 500 DPS
	float mag_conversion = 0.15; // typical sensitivity/conversion rate should be 0.15 μT/LSB

	// bias measurements
	float accel_bias[3];
	float gyro_bias[3];
	float mag_bias[3];

} ICM20948;

// updates SPI related struct elements so the SPI handle can interface with the IMU
void init_imu_spi(ICM20948 *imu, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *ncs_pin, uint16_t *ncs_pin_bank);

// general read/write to IMU register functions
uint8_t read_imu_reg(ICM20948 *imu, uint8_t reg_addr, uint8_t *data);
uint8_t write_imu_reg(ICM20948 *imu, uint8_t reg_addr, uint8_t data);
void write_mag_reg(ICM20948 *imu, uint8_t reg_addr, uint8_t data);
uint8_t read_mag_reg(ICM20948 *imu, uint8_t reg_addr);

// initialize the ICM20948
uint8_t who_am_i(ICM20948 *imu);
void imu_init(ICM20948 *imu);
void mag_init(ICM20948 *imu);

// functions to burst read vector component measurements from each sensor
uint8_t read_accel_vec(ICM20948 *imu);
uint8_t read_gyro_vec(ICM20948 *imu);
uint8_t read_mag_vec(ICM20948 *imu);

void calibrate_accel(ICM20948 *imu);
void calibrate_gyro(ICM20948 *imu);

#endif /* INC_ICM20948_H_ */
