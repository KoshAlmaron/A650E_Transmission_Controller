//Нештатный код/patch

#include <avr/interrupt.h>		// Прерывания.
#include <stdint.h>				// Коротние название int.
#include "tacho.h"				// Свой заголовок.
#include "configuration.h"		// Настройки.
#include "tcudata.h"			// Расчет и хранение всех необходимых параметров.

volatile uint16_t TachoImps = 0;

uint8_t TachoEW = 0;
uint16_t TachoTimer = 0;

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

void tacho_timer(uint8_t TimerAdd) {
	#ifndef USE_ENGINE_RPM
		return;
	#endif

	TachoTimer += TimerAdd;		// Счетчик времени.

	if (TachoTimer >= 500) {	// Расчет оборотов каждые 500 мс.
		uint16_t Imps = 0;
		cli();
			Imps = TachoImps;
			TachoImps = 0;
		sei();

		TCU.EngineRPM = Imps * 60;
		TachoTimer = 0;
	}

	// Поднятие флага EW при привышении оборотов.
	if (!TachoEW && TCU.EngineRPM >= ENGINE_ON_RPM_THRESHOLD) {TachoEW = 1;}

	// Если обороты ниже лимита, снимаем флаг EW.
	if (TachoEW && TCU.EngineRPM <= ENGINE_OFF_RPM_THRESHOLD) {TachoEW = 0;}
}

uint8_t get_tacho_ew() {
	return TachoEW;
}

// Обработчик прерывания для INT4
ISR (INT4_vect) {
	TachoImps++;
}


