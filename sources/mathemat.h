// Математические функции.
#ifndef _MATHEMAT_H_
	#define _MATHEMAT_H_

	uint16_t get_interpolated_value_uint8_t(uint16_t x, int16_t* ArrayX, uint8_t* ArrayY, uint8_t ArraySize);

	uint16_t get_interpolated_value_uint16_t(uint16_t x, int16_t* ArrayX, uint16_t* ArrayY, uint8_t ArraySize);
	int16_t get_interpolated_value_int16_t(int16_t x, int16_t* ArrayX, int16_t* ArrayY, uint8_t ArraySize);

#endif