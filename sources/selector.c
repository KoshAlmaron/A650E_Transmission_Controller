#include <stdint.h>		// Коротние название int.
#include <avr/io.h>		// Номера бит в регистрах.

#include "selector.h"	// Свой заголовок.
#include "tcudata.h"	// Расчет и хранение всех необходимых параметров.
#include "macros.h"		// Макросы.
#include "pinout.h"		// Список назначенных выводов.
#include "eeprom.h"		// Чтение и запись EEPROM.

// Счетчик для установки ошибки.
uint8_t ErrorTimer = 0;

// Настройка выводов для селектора.
void selector_init() {
	// Весь порт PA как вход с подтяжкой.
	SET_PORT_MODE(C, 0);
	PORT_WRITE(C, 0xFF);
	
	// Вход без подтяжки для педали тормоза.
	SET_PIN_MODE_INPUT(BREAK_PEDAL_PIN);
	SET_PIN_LOW(BREAK_PEDAL_PIN);

	// Вход без подтяжки для определение работы двигателя.
	SET_PIN_MODE_INPUT(ENGINE_WORK_PIN);
	SET_PIN_LOW(ENGINE_WORK_PIN);
}

// Текущая позиция селектора АКПП.
void selector_position() {
	// На селекторе АКПП 6 позиций + две дополнительные.
	// Это позволяет использовать один порт контроллера.
	uint8_t Val = ~PINC;
	switch (Val) {
		case 1:
			TCU.Selector = 1;	// P
			ErrorTimer = 0;
			break;
		case 2:
			TCU.Selector = 2;	// R
			ErrorTimer = 0;
			break;
		case 4:
			TCU.Selector = 3;	// N
			ErrorTimer = 0;
			break;
		case 8:
			TCU.Selector = 4;	// D
			ErrorTimer = 0;
			break;
		case 72:
			// D + 4.
			TCU.Selector = 5;	// 4
			ErrorTimer = 0;
			break;
		case 16:
			TCU.Selector = 6;	// 3
			ErrorTimer = 0;
			break;
		case 32:
			TCU.Selector = 7;	// 2
			ErrorTimer = 0;
			break;
		case 160:
			// 2 + L.
			TCU.Selector = 8;	// L
			ErrorTimer = 0;
			break;
		default:
			// Если четыре раза нет значения, значит ошибка.
			if (ErrorTimer < 4) {ErrorTimer++;}
			else {TCU.Selector = 9;}
	}
}

void engine_n_break_state() {
	// Педаль тормоза.
	TCU.Break = PIN_READ(BREAK_PEDAL_PIN) ? 1 : 0;

	// Флаг работы двигателя.
	uint8_t EW = PIN_READ(ENGINE_WORK_PIN) ? 1 : 0;
	// Сохранение настроек при выключении двигателя.
	if (TCU.EngineWork && !EW) {update_eeprom();}
	TCU.EngineWork = EW;
}