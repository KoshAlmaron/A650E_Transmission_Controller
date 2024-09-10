#include <avr/interrupt.h>		// Прерывания.
#include <stdint.h>				// Коротние название int.
#include "configuration.h"		// Настройки.
#include "spdsens.h"			// Свой заголовок.


// Минимальное сырое значения для фильтрации ошибочных значений.

// Шаг 4мкс (1/64).
// #define MIN_RAW_VALUE_OD 132
// #define MIN_RAW_VALUE_OUT 152

// Шаг 0.5мкс (1/8).
#define MIN_RAW_VALUE_OD 1380
#define MIN_RAW_VALUE_OUT 1380

// Размер буфера и размер битового сдвига для деления.
#define SENSOR_BUFFER_SIZE 32 + 4
#define SENSOR_BUFFER_SHIFT 5

// Кольцевые буферы для замера оборотов, прохождение 1 зуба шагах таймера (4 мкс).
volatile uint16_t DrumArray[SENSOR_BUFFER_SIZE] = {0};
volatile uint16_t OutputArray[SENSOR_BUFFER_SIZE] = {0};
// Текущая позиция в буфере.
volatile uint8_t DrumPos = 0;
volatile uint8_t OutputPos = 0;
// Признак готовности буфера для записи.
volatile uint8_t DrumBufferReady = 1;
volatile uint8_t OutputBufferReady = 1;

// Расчет скорости корзины овердрайва АКПП.
uint16_t get_overdrive_drum_rpm() {
	// Делитель 8
	// 1000000 * 60 / ([Шаг таймера, мкс] * [Кол-во зубов] * [Кол-во шагов])
	// Шаг таймера = 1000000 / [Частота таймера]
	// (1000000 * 60) / (4 * 16)
	//#define PRM_CALC_COEF_OD 937500UL	// Шаг 4мкс (1/64).
	#define PRM_CALC_COEF_OD 7500000UL	// Шаг 0.5мкс (1/8).

	DrumBufferReady = 0;	// Запрещаем обновление буфера в прерываниях.

	// Переменные для хранения двух крайних значений.
	uint16_t MaxValue = DrumArray[0];
	uint16_t MaxValuePrev = DrumArray[0];
	
	uint16_t MinValue = DrumArray[0];
	uint16_t MinValuePrev = DrumArray[0];

	uint32_t AVG = 0;
	// Суммируем среднее и ищем крайние значения.
	for (uint8_t i = 0; i < SENSOR_BUFFER_SIZE; i++) {
		AVG += DrumArray[i];

		if (DrumArray[i] < MinValue) {
			MinValuePrev = MinValue;
			MinValue = DrumArray[i];
		}
		if (DrumArray[i] > MaxValue) {
			MaxValuePrev = MaxValue;
			MaxValue = DrumArray[i];
		}
	}
	DrumBufferReady = 1;	// Разрешаем обновление буфера в прерываниях.

	// Исключаем два самых больших и два самых маленьких значения.
	AVG -= (MaxValue + MaxValuePrev + MinValue + MinValuePrev);
	// Находим среднее значение.
	AVG = AVG >> SENSOR_BUFFER_SHIFT;

	// Рассчитываем обороты вала.
	uint16_t RPM = 0;
	if (AVG < UINT16_MAX) {RPM = (uint32_t) PRM_CALC_COEF_OD / AVG;}
	return RPM;
}

// Расчет скорости выходного валов АКПП.
uint16_t get_output_shaft_rpm() {
	// 1000000 * 60 / ([Шаг таймера, мкс] * [Кол-во зубов] * [Кол-во шагов])
	// Шаг таймера = 1000000 / [Частота таймера]
	// (1000000 * 60) / (4 * 12)
	//#define PRM_CALC_COEF_OUT 1250000UL		// Шаг 4мкс (1/64).
	#define PRM_CALC_COEF_OUT 10000000UL		// Шаг 0.5мкс (1/8).

	OutputBufferReady = 0;	// Запрещаем обновление буфера в прерываниях.

	// Переменные для хранения двух крайних значений.
	uint16_t MaxValue = OutputArray[0];
	uint16_t MaxValuePrev = OutputArray[0];
	
	uint16_t MinValue = OutputArray[0];
	uint16_t MinValuePrev = OutputArray[0];

	uint32_t AVG = 0;
	// Суммируем среднее и ищем крайние значения.
	for (uint8_t i = 0; i < SENSOR_BUFFER_SIZE; i++) {
		AVG += OutputArray[i];

		if (OutputArray[i] < MinValue) {
			MinValuePrev = MinValue;
			MinValue = OutputArray[i];
		}
		if (OutputArray[i] > MaxValue) {
			MaxValuePrev = MaxValue;
			MaxValue = OutputArray[i];
		}
	}
	OutputBufferReady = 1;	// Разрешаем обновление буфера в прерываниях.

	// Исключаем два самых больших и два самых маленьких значения.
	AVG -= (MaxValue + MaxValuePrev + MinValue + MinValuePrev);
	// Находим среднее значение.
	AVG = AVG >> SENSOR_BUFFER_SHIFT;

	// Рассчитываем обороты вала.
	uint16_t RPM = 0;
	if (AVG < UINT16_MAX) {RPM = (uint32_t) PRM_CALC_COEF_OUT / AVG;}
	return RPM;
}

// Корзина овердрайва.
// Прерывание по захвату сигнала таймером 4.
ISR (TIMER4_CAPT_vect) {
	TCNT4 = 0;				// Обнулить счётный регистр.

	if (DrumBufferReady && ICR4 > MIN_RAW_VALUE_OD) {
		// Записываем значение в кольцевой буфер.	
		DrumArray[DrumPos] = ICR4;
		DrumPos++;
		if (DrumPos >= SENSOR_BUFFER_SIZE) {DrumPos = 0;}
	}
}
// Прерывание по переполнению таймера 4.
ISR (TIMER4_OVF_vect) {
	for (uint8_t i = 0; i < SENSOR_BUFFER_SIZE; i++) {
		DrumArray[i] = UINT16_MAX;
	}
}

// Выходной вал.
// Прерывание по захвату сигнала таймером 5.
ISR (TIMER5_CAPT_vect) {
	TCNT5 = 0;						// Обнулить счётный регистр.

	if (OutputBufferReady && ICR5 > MIN_RAW_VALUE_OUT) {
		// Записываем значение в кольцевой буфер.	
		OutputArray[OutputPos] = ICR5;
		OutputPos++;
		if (OutputPos >= SENSOR_BUFFER_SIZE) {OutputPos = 0;}
	}
}
// Прерывание по переполнению таймера 5.
ISR (TIMER5_OVF_vect) {
	for (uint8_t i = 0; i < SENSOR_BUFFER_SIZE; i++) {
		OutputArray[i] = UINT16_MAX;
	}
}