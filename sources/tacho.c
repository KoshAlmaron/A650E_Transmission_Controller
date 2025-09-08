#include <avr/interrupt.h>		// Прерывания.
#include <stdint.h>				// Коротние название int.
#include "tacho.h"				// Свой заголовок.
#include "configuration.h"		// Настройки.

uint16_t TachoTimer = 0;
volatile uint16_t TachoImps = 0;
volatile uint16_t TachoRPM = 0;

// Инициализация внешнего прерывания INT4 по спаду фронта.
void tacho_init() {
	#ifndef USE_ENGINE_RPM
		return;
	#endif

	DDRE &= ~(1 << 4);		// Настраиваем PE4 как вход с подтяжкой.
	PORTE |= (1 << 4);

	EIMSK |= (1 << INT4);		// Включаем внешнее прерывание INT4.
	EICRB |= (1 << ISC41);		// Настраиваем триггер по спаду фронта.
}

void tacho_timer() {
	#ifndef USE_ENGINE_RPM
		return;
	#endif

	TachoTimer++;
	if (TachoTimer >= 500) {
		TachoTimer = 0;
		TachoImps = 0;
	}
}

uint16_t tacho_get_rpm() {
	uint16_t RPM = 0;
	cli();
		RPM = TachoRPM;
	sei();
	return RPM;
}

// Обработчик прерывания для INT4
ISR (INT4_vect) {
	if (TachoTimer >= 250) {
		TachoRPM = TachoImps * 120;
		TachoTimer = 0;
		TachoImps = 0;
	}
	else {
		TachoImps++;
	}
}

