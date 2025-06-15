#include "spi.h"
#include "stm32f4xx.h"

void SPI_pins_init() {

	RCC->AHB1ENR |= RCC_AH1BRENR_GPIOEN; //enable clock for GPIOA

	GPIOB->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5); // zeroing register bits in preparation for potential overwrite
	GPIOB->MODER |= GPIO_MODER_MODE4_1| GPIO_MODER_MODE5_1; //set PB4 and PB5 to alternate function mode

	GPIOC->MODER &= ~(GPIO_MODER_MODE10); // zeroing register bits in preparation for potential overwrite
	GPIOC->MODER |= GPIO_MODER_MODE10; // set PC10 to alternate function mode

	// Set Alternate Function 6 (AF6） for SPI3 on PB4, PB5, and PC10
	// AF[0] for pins 0-7, AF[1] for pins 8-15
	// Bit shift is a multiple of 4 as each pin has 4 bits to define a function
	// PB4 = AFR[0], index = 4 * 4 = 16
	// PB5 = AFR[0], index = 5 * 4 = 20
	// PC10 = AFR[1], index = (10-8)*4 = 8

	GPIOB->AFR[0] |= ~((0xF << 16) | (0xF << 20)); // zeroing register bits in preparation for potential overwrite
	GPIOB->AFR[0] |= ((0x6 << 16) | (0x6 << 20)); // setting PB4 and PB5 to AF6 (SPI MISO and MOSI)
	GPIOC->AFR[1] |= ~(0xF << 8); // zeroing register bits in preparation for potential overwrite
	GPIOC->AFR[1] |= (0x6 << 8); // setting PC10 to AF6 (SPI3 SCK)

	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN; // enable clock to access SPI3

	SPI3->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; //software peripheral management, allows software control over NCS
	SPI3->CR1 |= SPI_CR1_MSTR; // set SPI as controller mode
	SPI3->CR1 |= SPI_CR1_SPE; // enable SPI peripheral

}

void NCS_pins_init() {

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable clock for GPIOA

	// set PA8 and PA9 to Output Mode
	GPIOA->MODER |= GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0;
	GPIOA->MODER &= ~(GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);

	GPIOA->BSRR = GPIO_BSRR_BS8 | GPIO_BSRR_BS9; // Set PA8 to PA9 to high state

}
