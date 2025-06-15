#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "stdint.h"

typedef enum {
	ICM20948_NCS = 0,
	BMP390_NCS = 1
} peripheral_select;

void SPI_pins_init();

void NCS_pins_init();

void SPI_configure();

void SPI_write(uint8_t *data, uint32_t size);

void SPI_read(uint8_t *data, uint32_t size);

void peripheral_select(peripheral_select peripheral);

void peripheral_deselect(peripheral_select peripheral);

#endif /* INC_SPI_H_ */
