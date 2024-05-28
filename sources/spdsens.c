#include <avr/interrupt.h>		// Прерывания.
#include <stdint.h>				// Коротние название int.
#include "spdsens.h"			// Свой заголовок.

// Для датчиков используются кольцевые буферы усреднения.
// Значения в массивах хранятся в числах АЦП - 0..1023.

// Размер буфера и размер битового сдвига для деления.
#define SENSOR_BUFFER_SIZE 16
#define SENSOR_BUFFER_SHIFT 4

// Измеренный период сигнала в шагах таймера (4 мкс).
volatile uint16_t OverDriveDrumPeriod = UINT16_MAX;
volatile uint16_t OutputShaftPeriod = UINT16_MAX;
// Кольцевые буферы для замера оборотов, прохождение 1 зуба шагах таймера (4 мкс).
uint16_t OverDriveDrumArray[SENSOR_BUFFER_SIZE] = {0};
uint16_t OutputShaftArray[SENSOR_BUFFER_SIZE] = {0};
// Текущая позиция в буфере.
uint8_t OverDriveDrumPos = 0;
uint8_t OutputShaftPos = 0;

// Расчет скорости корзины овердрайва АКПП.
void calculate_overdrive_drum_rpm() {
	// 1000000 * 60 / ([Шаг таймера, мкс] * [Кол-во зубов] * [Кол-во шагов])
	// Шаг таймера = 1000000 / [Частота таймера]
	// (1000000 * 60) / (4 * 12)
	#define SPEED_CALC_COEF 10000000UL	// Таймер x8

	// Забираем значение из переменной с прерываниями.
	uint16_t Period = 0;
	cli();
	Period = OverDriveDrumPeriod;
	sei();

	// Рассчитываем обороты вала.
	uint16_t RPM = 0;
	if (Period != UINT16_MAX) {
		RPM = (uint32_t) SPEED_CALC_COEF / Period;
	}

	// Записываем значение в кольцевой буфер
	OverDriveDrumArray[OverDriveDrumPos] = RPM;
	OverDriveDrumPos++;
	if (OverDriveDrumPos >= SENSOR_BUFFER_SIZE) {OverDriveDrumPos = 0;}
}

// Расчет скорости выходного валов АКПП.
void calculate_output_shaft_rpm() {
	// 1000000 * 60 / ([Шаг таймера, мкс] * [Кол-во зубов] * [Кол-во шагов])
	// Шаг таймера = 1000000 / [Частота таймера]
	// (1000000 * 60) / (4 * 12)
	#define SPEED_CALC_COEF 10000000UL	// Таймер x8

	// Забираем значение из переменной с прерываниями.
	uint16_t Period = 0;
	cli();
	Period = OutputShaftPeriod;
	sei();

	// Рассчитываем обороты вала.
	uint16_t RPM = 0;
	if (Period != UINT16_MAX) {
		RPM = (uint32_t) SPEED_CALC_COEF / Period;
	}

	// Записываем значение в кольцевой буфер
	OutputShaftArray[OutputShaftPos] = RPM;
	OutputShaftPos++;
	if (OutputShaftPos >= SENSOR_BUFFER_SIZE) {OutputShaftPos = 0;}
}

// Вызывается переодически из цикла.
void speed_sensors_read() {
	calculate_overdrive_drum_rpm();
	calculate_output_shaft_rpm();
}

uint16_t get_overdrive_drum_rpm() {
	// Находим среднее значение.
	uint32_t AVG = 0;
	for (uint8_t i = 0; i < SENSOR_BUFFER_SIZE; i++) {AVG += OverDriveDrumArray[i];}
	return AVG >> SENSOR_BUFFER_SHIFT;
}

uint16_t get_output_shaft_rpm() {
	// Находим среднее значение.
	uint32_t AVG = 0;
	for (uint8_t i = 0; i < SENSOR_BUFFER_SIZE; i++) {AVG += OutputShaftArray[i];}
	return AVG >> SENSOR_BUFFER_SHIFT;
}

// Корзина овердрайва.
// Прерывание по захвату сигнала таймером 4.
ISR (TIMER4_CAPT_vect) {
	TCNT4 = 0;						// Обнулить счётный регистр.
	OverDriveDrumPeriod = ICR4;		// Результат из регистра захвата.
}
// Прерывание по переполнению таймера 4.
ISR (TIMER4_OVF_vect) {OverDriveDrumPeriod = UINT16_MAX;}

// Выходной вал.
// Прерывание по захвату сигнала таймером 5.
ISR (TIMER5_CAPT_vect) {
	TCNT5 = 0;						// Обнулить счётный регистр.
	OutputShaftPeriod = ICR5;		// Результат из регистра захвата.
}
// Прерывание по переполнению таймера 5.
ISR (TIMER5_OVF_vect) {OutputShaftPeriod = UINT16_MAX;}