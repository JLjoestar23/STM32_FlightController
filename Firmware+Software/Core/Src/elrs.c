/*
 * elrs.c
 *
 *  Created on: Aug 8, 2025
 *      Author: jliu3
 */
#include "elrs.h";

void ELRS_init(ELRS *Rx, UART_HandleTypeDef *huart) {
	HAL_UART_Recieve_DMA(huart, Rx->RxBuffer, 2); // initialize DMA transaction to start filling 2 byte buffer
}

void HAL_UART_RxHalfCpltCallback(ELRS *Rx, UART_HandleTypeDef *huart) {
	ELRS_parse_byte(&Rx, Rx->RxBuffer[0]); // parse the first byte of the buffer
}

void HAL_UART_RxCpltCallback(ELRS *Rx, UART_HandleTypeDef *huart) {
	ELRS_parse_byte(&Rx, Rx->RxBuffer[1]); // parse the second byte of the buffer
	HAL_UART_Recieve_DMA(huart, Rx->RxBuffer, 2); // reset the DMA transaction to continuously repeat the process
}

void ELRS_parse_byte(ELRS *Rx, uint8_t RxByte) {
	// if the byte is the device address byte AND there is no current temporary buffer, start a new temporary buffer
	if (Rx->temp_index == 0) {
		if (RxByte == DEVICE_ADDRESS) {
			Rx->temp_buffer[Rx->temp_index] = RxByte;
		} else {
			Rx->temp_index = 0;
		}
	}

	// second byte should be equal to the frame length
	// frame length = payload type byte + payload bytes + CRC byte = 24 bytes
	// assigns frame_length the recieved byte value
	if (Rx->temp_index == 1) {
		Rx->frame_length = RxByte;
		if (Rx->frame_length != 24) {  // Critical check for 16 channels
			Rx->temp_index = 0;  // Reset if invalid
			return;
		}
		Rx->temp_buffer[Rx->temp_index++] = RxByte; // index to the next array element
	}

	// once past the frame length, receive the following bytes: payload type, payload, and CRC bytes
	if (Rx->temp_index > 1 && Rx->temp_index < Rx->frame_length + 2) {
		if (Rx->temp_index >= ELRS_PACKET_SIZE) {
			Rx->temp_index = 0;  // Reset on overflow
			return;
		}
		Rx->temp_buffer[Rx->temp_index++] = RxByte;
	}

	// Cyclic Redundancy Check (CRC) on the last byte
	if (Rx->temp_index == Rx->frame_length + 1) {
		uint8_t crc = crc8(&Rx->temp_buffer[2], Rx->frame_length);
		// if CRC is successful, then set the data status to ready, meaning the buffer is valid
		if (crc == RxByte) {
			memcpy(Rx->full_buffer, Rx->temp_buffer, ELRS_PACKET_SIZE);
			Rx->data_status = DATA_STATUS_READY;
		}
		Rx->temp_index = 0; // reset the temporary index to 0 to start a new buffer
	}
}

void assign_channels(ELRS *Rx) {
	// if invalid data is received, do not assign new channel values
	if (Rx->data_status != DATA_STATUS_READY) return;

	// channel payload starts at the 3rd index (skip sync, length, and type bytes)
	uint8_t *payload = &Rx->temp_buffer[3];  // pointer to channel value payload (22 bytes)

	// since each channel are 11 bit values, bit shifting is necessary for correct values
	// payload format is LSB first, following bytes are MSB
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

	// reset the data ready status
	Rx->data_status = DATA_STATUS_NREADY;
}

uint8_t crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++) {
    	crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}
