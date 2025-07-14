/*
 * icm20948.h
 *
 *  Created on: Jun 17, 2025
 *      Author: jliu3
 */

#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_

#include "stdint.h"
#include "spi.h"
#include "stm32f4xx.h"

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

// user bank 2
#define GYRO_SMPL_RATE			0x00
#define GYRO_CFG_1				0x01
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
#define BEGIN_VEC_DATA			0x11
#define MAG_CTRL_2				0x31
#define MAG_CTRL_3				0x32



// structures
struct vec3 {
	float x;
	float y;
	float z;
};

struct quat4 {
	float a;
	float b;
	float c;
	float d;
};

typedef struct {

	// conversion constants to be applied to raw measurements
	float accel_conversion = 1/8192; // sensitivity scale factor is 8192 LSB/g
	float gyro_conversion = 1/65.5; // sensitivity scale factor is 65.5 LSB/dps
	float mag_conversion = 0.15; // typical sensitivity/conversion rate should be 0.15 μT/LSB

	// x, y, z component measurements
	float A[3];
	float G[3];
	float M[3];

} imu;

// general read/write to register functions
uint8_t read_imu_reg(uint8_t reg_addr, uint8_t *data);

uint8_t write_imu_reg(uint8_t reg_addr, uint8_t *data);

// initialize the ICM20948
uint8_t imu_init(void);

// functions to read vector component measurements from each sensor
uint8_t read_accel_vec();

uint8_t read_gyro_vec();

uint8_t read_mag_vec();

#endif /* INC_ICM20948_H_ */
