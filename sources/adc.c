#include <stdint.h>			// Коротние название int.
#include <avr/io.h>			// Номера бит в регистрах.
#include "adc.h"			// Свой заголовок.

// Размер буфера и размер битового сдвига для деления.
#define ADC_BUFFER_SIZE 8
#define ADC_BUFFER_SHIFT 3

// Стандартное кол-во каналов.
#define ADC_CHANNEL_STD 2
// Максимальное количество каналов для измерений.
#define ADC_CHANNEL_MAX 5

// Количество активных каналов.
uint8_t ChannelsCount = ADC_CHANNEL_STD;

// Список каналов АЦП и текущая позиция.
uint8_t Channels[ADC_CHANNEL_MAX] = {0, 1, 11, 12, 13};
uint8_t ChPos = 0;

// Измеренные значения с буфером усреднения.
uint16_t ADCValues[ADC_CHANNEL_MAX][ADC_BUFFER_SIZE] = {0};
// Текущая позиция в буфере.
uint8_t BufPos = 0;

// Инициализация АЦП.
void adc_init() {
	// Настройка всех портов с АЦП как вход без подтяжки.
	DDRF = 0;
	DDRK = 0;
	PORTF = 0;
	PORTK = 0;

	ADMUX = 0;
	ADCSRA = 0;
	ADCSRB = 0;

	ADMUX |= (1 << REFS0);					// Опорное напряжение 5В.
	ADCSRA |= (1 << ADEN);					// Включаем АЦП.
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1);	// Предделитель 64.
}

void adc_read() {
	// Измерения проходят в цикле, сначала все каналы в первую ячейку буфера,
	// потом все каналы во вторую и т.д.

	// Если бит ADSC в регистре ADCSRA сброшен,
	// то можно запускать следующее измерение.
	if (!(ADCSRA & (1 << ADSC))) {
		// Считываем значение из регистров.
		ADCValues[ChPos][BufPos] = ADCL | (ADCH << 8);

		// Переходим к следующему каналу.
		ChPos++;
		if (ChPos >= ChannelsCount) {
			ChPos = 0;
			BufPos++;
			if (BufPos >= ADC_BUFFER_SIZE) {BufPos = 0;}
		}
		// Сброс канала ADC
		ADMUX &= ~(1 << MUX0);
		ADMUX &= ~(1 << MUX1);
		ADMUX &= ~(1 << MUX2);
		// Установка текущего канала.
		// На Atmega2560 бит MUX5 переключает выбор канала,
		// 0 - 0...7
		// 1 - 8...15
		if (Channels[ChPos] < 8) {
			ADCSRB &= ~(1 << MUX5);
			ADMUX |= Channels[ChPos];
		}
		else {
			ADCSRB |= (1 << MUX5);
			ADMUX |= (Channels[ChPos] - 8);
		}
		// Запуск измерения.
		ADCSRA |= (1 << ADSC);
	}
}

uint16_t get_adc_value(uint8_t Channel) {
	// Находим среднее значение.
	uint32_t AVG = 0;
	for (uint8_t i = 0; i < ADC_BUFFER_SIZE; i++) {AVG += ADCValues[Channel][i];}
	return AVG >> ADC_BUFFER_SHIFT;
}

void add_channels_on(uint8_t Value) {
	if (Value) {
		ChannelsCount = ADC_CHANNEL_MAX;
	}
	else {
		ChannelsCount = ADC_CHANNEL_STD;
	}
}