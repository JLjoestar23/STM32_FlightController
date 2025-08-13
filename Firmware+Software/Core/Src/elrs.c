/*
 * elrs.c
 *
 *  Created on: Aug 8, 2025
 *      Author: jliu3
 */
#include "elrs.h";

void ELRS_init(ELRS *Rx, UART_HandleTypeDef *huart) {
	HAL_UART_Recieve_DMA(huart, Rx->RxBuffer, 2);
}

void HAL_UART_RxHalfCpltCallback(ELRS *Rx, UART_HandleTypeDef *huart) {
	ELRS_parse_byte(&Rx, Rx->RxBuffer[0]);
}

void HAL_UART_RxCpltCallback(ELRS *Rx, UART_HandleTypeDef *huart) {
	ELRS_parse_byte(&Rx, Rx->RxBuffer[1]);
	HAL_UART_Recieve_DMA(huart, Rx->RxBuffer, 2);
}

void ELRS_parse_byte(ELRS *Rx, uint8_t RxByte) {
	if (Rx->temp_index == 0) {
		if (RxByte == DEVICE_ADDRESS) {
			Rx->temp_buffer[Rx->temp_index] = RxByte;
		} else {
			Rx->temp_index = 0;
		}
	}

	if (Rx->temp_index == 1) {
		Rx->frame_length = RxByte;
		if (Rx->frame_length != 24) {  // Critical check for 16 channels
			Rx->temp_index = 0;  // Reset if invalid
			return;
		}
		Rx->temp_buffer[Rx->temp_index++] = RxByte;
	}

	if (Rx->temp_index > 1 && Rx->temp_index < Rx->frame_length + 2) {
		if (Rx->temp_index >= ELRS_PACKET_SIZE) {
			Rx->temp_index = 0;  // Reset on overflow
			return;
		}
		Rx->temp_buffer[Rx->temp_index++] = RxByte;
	}

	if (Rx->temp_index == Rx->frame_length + 1) {
		uint8_t crc = crc8(&Rx->temp_buffer[2], Rx->frame_length);
		if (crc == RxByte) {
			memcpy(Rx->full_buffer, Rx->temp_buffer, ELRS_PACKET_SIZE);
			Rx->data_status = DATA_STATUS_READY;
		}
		Rx->temp_index = 0;
	}
}

void assign_channels(ELRS *Rx) {
	if (Rx->data_status != DATA_STATUS_READY) return;

	// channel payload starts at the 3rd index (skip sync, length, and type bytes)
	uint8_t *payload = &Rx->full_buffer[3];  // pointer to channel value payload (22 bytes)

	Rx->channels[0] = (payload[3] | payload[4] << 8) & 0x07FF;
	Rx->channels[1] = (payload[4] >> 3 | payload[5] << 5) & 0x07FF;
	Rx->channels[2] = (payload[5] >> 6 | payload[6] << 2 | payload[7] << 10) & 0x07FF;
	Rx->channels[3] = (payload[7] >> 1 | payload[8] << 7) & 0x07FF;
	Rx->channels[4] = (payload[8] >> 4 | payload[9] << 4) & 0x07FF;
	Rx->channels[5] = (payload[9] >> 7 | payload[10] << 1 | payload[11] << 9) & 0x07FF;
	Rx->channels[6] = (payload[11] >> 2 | payload[12] << 6) & 0x07FF;
	Rx->channels[7] = (payload[12] >> 5 | payload[13] << 3) & 0x07FF;
	Rx->channels[8] = (payload[14] | payload[15] << 8) & 0x07FF;
	Rx->channels[9] = (payload[15] >> 3 | payload[16] << 5) & 0x07FF;
	Rx->channels[10] = (payload[16] >> 6 | payload[17] << 2 | payload[18] << 10) & 0x07FF;
	Rx->channels[11] = (payload[18] >> 1 | payload[19] << 7) & 0x07FF;
	Rx->channels[12] = (payload[19] >> 4 | payload[20] << 4) & 0x07FF;
	Rx->channels[13] = (payload[20] >> 7 | payload[21] << 1 | Rx->payload[22] << 9) & 0x07FF;
	Rx->channels[14] = (payload[22] >> 2 | payload[23] << 6) & 0x07FF;
	Rx->channels[15] = (payload[23] >> 5 | payload[24] << 3) & 0x07FF;

	Rx->data_status = DATA_STATUS_NREADY;
}

uint8_t crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++) {
    	crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}
