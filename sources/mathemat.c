#include <stdint.h>			// Коротние название int.

#include "mathemat.h"		// Свой заголовок.

// Возвращаент интерполированное значение uint16_t из графика.
uint16_t get_interpolated_value_uint16_t(uint16_t x, int16_t* ArrayX, uint16_t* ArrayY, uint8_t ArraySize) {
	uint16_t Result = 0;

	if (x <= ArrayX[0]) {return ArrayY[0] ;}
	if (x >= ArrayX[ArraySize - 1]) {return ArrayY[ArraySize - 1];}

	// Находим позицию в графике.
	for (uint8_t i = 0; i < ArraySize; i++) {
		if (x <= ArrayX[i]) {
			// Находим значение с помощью интерполяции.
			uint16_t x0 = ArrayX[i - 1];
			uint16_t x1 = ArrayX[i];
			
			uint16_t y0 = ArrayY[i - 1];
			uint16_t y1 = ArrayY[i];

			// Переворачиваем значения, если Y идет на уменьшение.
			if (y0 > y1) {
				y0 = ArrayY[i];
				y1 = ArrayY[i - 1];
				x = x0 + x1 - x;
			}

			Result = y0 * 16 + (((y1 - y0) * 16) * (x - x0)) / (x1 - x0);
			break;
		}
	}
	Result /= 16;
	return Result;
}

// Возвращаент интерполированное значение int16_t из графика
int16_t get_interpolated_value_int16_t(int16_t x, int16_t* ArrayX, int16_t* ArrayY, uint8_t ArraySize) {
	int16_t Result = 0;

	// Проверка в каком направлении идет увеличение значений в массиве.
	int8_t Reverse = 0;
	if (ArrayX[0] > ArrayX[ArraySize - 1]) {Reverse = 1;}

	// Значение за пределами графика.
	if (Reverse) {
		if (x >= ArrayX[0]) {return ArrayY[0];}
		if (x <= ArrayX[ArraySize - 1]) {return ArrayY[ArraySize - 1];}
	}
	else {
		if (x <= ArrayX[0]) {return ArrayY[0];}
		if (x >= ArrayX[ArraySize - 1]) {return ArrayY[ArraySize - 1];}
	}

	// Находим позицию в графике.
	for (uint8_t i = 0; i < ArraySize; i++) {
		if ((Reverse && x >= ArrayX[i]) || (!Reverse && x <= ArrayX[i])) {
			// Находим значение с помощью интерполяции.
			int16_t x0 = ArrayX[i - 1];
			int16_t x1 = ArrayX[i];
			
			int16_t y0 = ArrayY[i - 1];
			int16_t y1 = ArrayY[i];

			Result = y0 * 16 + (((y1 - y0) * 16) * (x - x0)) / (x1 - x0);
			break;
		}
	}
	Result /= 16;
	return Result;
}