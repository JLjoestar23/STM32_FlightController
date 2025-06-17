#include "spi.h"
#include "stm32f4xx.h"


/*
 * @brief Initializes which GPIO pins are to be utilized in SPI mode
 */
void SPI_pins_init() {
	/*
	 * RCC_AH1BRENR_GPIOEN is a bit mask macro for the register location that enables the clock
	 * GPIO_MODER_MODEx_y is a bit mask macro for the register location that determines operating mode of pin x by bit y
	 */
	RCC->AHB1ENR |= RCC_AH1BRENR_GPIOEN; //enable clock for GPIOA

	// set PB4 and PB5 to alternate function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0);
	GPIOB->MODER |= GPIO_MODER_MODE4_1| GPIO_MODER_MODE5_1;

	// set PC10 to alternate function mode
	GPIOC->MODER &= ~(GPIO_MODER_MODE10_0);
	GPIOC->MODER |= GPIO_MODER_MODE10_1;

	/*
	 * Set Alternate Function 6 (AF6） for SPI3 on PB4, PB5, and PC10
	 * AF[0] for pins 0-7, AF[1] for pins 8-15
	 * Bit shift is a multiple of 4 as each pin has 4 bits to define a function
	 * PB4 = AFR[0], index = 4 * 4 = 16
	 * PB5 = AFR[0], index = 5 * 4 = 20
	 * PC10 = AFR[1], index = (10-8)*4 = 8
	 */

	GPIOB->AFR[0] |= ~((0xF << 16) | (0xF << 20)); // zeroing register bits in preparation for potential overwrite
	GPIOB->AFR[0] |= ((0x6 << 16) | (0x6 << 20)); // setting PB4 and PB5 to AF6 (SPI MISO and MOSI)
	GPIOC->AFR[1] |= ~(0xF << 8); // zeroing register bits in preparation for potential overwrite
	GPIOC->AFR[1] |= (0x6 << 8); // setting PC10 to AF6 (SPI3 SCK)

}

/*
 * @brief Initializes which GPIO pins are to be utilized as not chip select pins
 */
void NCS_pins_init() {
	/*
	 * Uses similar macros to SPI_pins_init()
	 * GPIO_BSRR_BSx are bit mask macros for the Bit Set and Reset register that sets them high or low
	 */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable clock for GPIOA

	// set PA8 and PA9 to Output Mode
	GPIOA->MODER |= GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0;
	GPIOA->MODER &= ~(GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);

	GPIOA->BSRR = GPIO_BSRR_BS8 | GPIO_BSRR_BS9; // Set PA8 to PA9 to high state

}


/*
 * @brief Configures and enables the SPI pins with a baud rate to 5.25 MBits/s, full duplex, MSB first, and 16 bit data frame
 * No bit mask macros used here, just manual bit shifting
 */
void SPI_configure() {

	RCC->AHB1ENR |= RCC_APB1ENR_SPI3EN; // enable clock for SPI3

	// set clock to fPCLK/8 (5.25 MBits/s)
	// according to data sheet: change bits 3,4,5 on CR1 so that BR[2:0] = 010
	SPI3->CR1 &= ~(0x1U << 3);
	SPI3->CR1 |= (0x1U << 4);
	SPI3->CR1 &= ~(0x1U << 5);

	SPI3->CR1 &= ~(0x1U << 10); // enable full duplex communication

	SPI3->CR1 &= ~(0x1U << 7); // set MSB first

	SPI3->CR1 |= (0x1U << 2); // set mode to controller

	SPI3->CR1 &= (0x1U << 11); // set data frame format to 16 bits

	// select software peripheral management
	SPI3->CR1 |= (0x1U << 8);
	SPI3->CR1 |= (0x1U << 9);

	// enable SPI by setting enable register to 1
	SPI1->CR1 |= (0x1U << 6);

}

/*
 * @brief Transmit an amount of data in blocking mode
 * @params TxData: pointer to the transmit data buffer
 * @params size: amount of data elements
 * @return SPI_OK, SPI_ERROR, or SPI_TIMEOUT
 */
SPI_status SPI_write(uint8_t *TxData, uint32_t size) {

	uint32_t i = 0;
	uint32_t start;

	if ((TxData == NULL) || (size == 0U)) {
		return SPI_ERROR;
	}

	while(i < size) {
		// wait until the transmit buffer (TXE) is empty, return a timeout if necessary
		start = HAL_getTick();
		while(!(SPI3->SR & (SPI_SR_TXE))) {
			if ((HAL_getTick() - start) > timeout_ms) return SPI_TIMEOUT;
		}

		SPI3->DR = (uint8_t) TxData[i]; // write data to data register
		i++; // increment
	}

	// wait until TXE is set
	start = HAL_getTick();
	while(!(SPI3->SR & (SPI_SR_TXE))) {
		if ((HAL_getTick() - start) > timeout_ms) return SPI_TIMEOUT;
	}

	// wait for busy flag to reset, return timeout if necessary
	start = HAL_getTick();
	while((SPI3->SR & (SPI_SR_BSY))) {
		if ((HAL_getTick() - start) > timeout_ms) return SPI_TIMEOUT;
	}

	// clear overrun (OVR) flag
	(void) SPI3->DR;
	(void) SPI3->SR;

	return SPI_OK;
}

/*
 * @brief Receive an amount of data in blocking mode
 * @params RxData: pointer to the receive data buffer
 * @params size: amount of data elements
 * @return SPI_OK, SPI_ERROR, or SPI_TIMEOUT
 */
SPI_status SPI_read(uint8_t *RxData, uint32_t size) {

	uint32_t start;

	if ((RxData == NULL) || (size == 0U)) {
		    return SPI_ERROR;
	}

	while(size) {
		SPI3->DR = 0xFF; // send dummy data

		// wait for Receive Buffer Not Empty (RXNE) to be set, return timeout if necessary
		start = HAL_getTick();
		while(!(SPI3->SR & (SPI_SR_RXNE))) {
			if ((HAL_getTick() - start) > timeout_ms) return SPI_TIMEOUT;
		}

		*RxData++ = (SPI->DR); // read data from the register
		size--; // increment
	}
}

/**
 * @brief Full-duplex SPI read/write function in blocking mode
 * @param TxData: pointer to the transmit data buffer
 * @param RxData: pointer to the receive data buffer
 * @param size: amount of data elements
 * @param timeout_ms: timeout duration in ms
 * @returns SPI_OK, SPI_ERROR, or SPI_TIMEOUT
 */
SPI_status SPI_read_write(const uint8_t *TxData, uint8_t *RxData, uint32_t size, uint32_t timeout_ms) {

	uint32_t i = 0;
	uint32_t start;

	if ((TxData == NULL) || (RxData == NULL) || (size == 0U)) {
	    return SPI_ERROR;
	}

	while(i < size) {
		// wait until the transmit buffer (TXE) is empty, return a timeout if necessary
		start = HAL_getTick();
		while(!(SPI3->SR & (SPI_SR_TXE))) {
			if ((HAL_getTick() - start) > timeout_ms) return SPI_TIMEOUT;
		}

		SPI3->DR = (uint8_t) TxData[i]; // write data to data register

		// wait for Receive Buffer Not Empty (RXNE) to be set, return timeout if necessary
		start = HAL_getTick();
		while(!(SPI3->SR & (SPI_SR_RXNE))) {
			if ((HAL_getTick() - start) > timeout_ms) return SPI_TIMEOUT;
		}

		RxData[i] = SPI3->DR; // read data from the data register

		i++; // increment
	}

	// wait until TXE is set
	start = HAL_getTick();
	while(!(SPI3->SR & (SPI_SR_TXE))) {
		if ((HAL_getTick() - start) > timeout_ms) return SPI_TIMEOUT;
	}

	// wait for busy flag to reset, return timeout if necessary
	start = HAL_getTick();
	while((SPI3->SR & (SPI_SR_BSY))) {
		if ((HAL_getTick() - start) > timeout_ms) return SPI_TIMEOUT;
	}

	// clear overrun (OVR) flag
	(void) SPI3->DR;
	(void) SPI3->SR;

	return SPI_OK;
}

/*
 * @brief Controls NCS low
 * @params peripheral: enum type that defines which peripheral to begin data transfer with
 */
void peripheral_select(peripheral_select peripheral) {
	switch(peripheral) {
		// pulls either PB9 or PB8 low depending on which sensor the controller wants to begin communication with
		case ICM20948_NCS: GPIOB->BSRR = GPIO_BSRR_BR9; break;
		case BMP390_NCS: GPIOB->BSRR = GPIO_BSRR_BR8; break;
		default: break;
	}
}

/*
 * @brief Controls NCS high
 * @params peripheral: enum type that defines which peripheral to end data transfer with
 */
void peripheral_deselect(peripheral_select peripheral) {
	switch(peripheral) {
		//Pulls either PB9 or PB8 high depending on which sensor the controller wants to end communication with
		case ICM20948_NCS: GPIOB->BSRR = GPIO_BSRR_BS9; break;
		case BMP390_NCS: GPIOB->BSRR = GPIO_BSRR_BS8; break;
		default: break;
	}
}
