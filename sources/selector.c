#include <stdint.h>			// Коротние название int.
#include <avr/io.h>			// Номера бит в регистрах.

#include "selector.h"		// Свой заголовок.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "macros.h"			// Макросы.
#include "pinout.h"			// Список назначенных выводов.
#include "eeprom.h"			// Чтение и запись EEPROM.
#include "configuration.h"	// Настройки.

// Счетчик для установки ошибки.
uint8_t ErrorTimer = 0;

// Прототипы локальных функций.
static uint8_t get_selector_byte();

// Настройка выводов для селектора.
void selector_init() {
	// Все пины селектора как вход с подтяжкой.
	SET_PIN_MODE_INPUT(SELECTOR_P_PIN);
	SET_PIN_MODE_INPUT(SELECTOR_R_PIN);
	SET_PIN_MODE_INPUT(SELECTOR_N_PIN);
	SET_PIN_MODE_INPUT(SELECTOR_D_PIN);
	SET_PIN_MODE_INPUT(SELECTOR_3_PIN);
	SET_PIN_MODE_INPUT(SELECTOR_2_PIN);
	SET_PIN_MODE_INPUT(SELECTOR_4_PIN);
	SET_PIN_MODE_INPUT(SELECTOR_L_PIN);

	SET_PIN_HIGH(SELECTOR_P_PIN);
	SET_PIN_HIGH(SELECTOR_R_PIN);
	SET_PIN_HIGH(SELECTOR_N_PIN);
	SET_PIN_HIGH(SELECTOR_D_PIN);
	SET_PIN_HIGH(SELECTOR_3_PIN);
	SET_PIN_HIGH(SELECTOR_2_PIN);
	SET_PIN_HIGH(SELECTOR_4_PIN);
	SET_PIN_HIGH(SELECTOR_L_PIN);
	
	// Вход для педали тормоза.
	SET_PIN_MODE_INPUT(BREAK_PEDAL_PIN);
	#ifdef INVERSE_BREAK_PEDAL
		SET_PIN_HIGH(BREAK_PEDAL_PIN);	// Поддтяжка при инверсии.
	#else
		SET_PIN_LOW(BREAK_PEDAL_PIN);	// Без подтяжки.
	#endif

	// Вход без подтяжки для определение работы двигателя.
	SET_PIN_MODE_INPUT(ENGINE_WORK_PIN);
	SET_PIN_LOW(ENGINE_WORK_PIN);
}

static uint8_t get_selector_byte() {
	uint8_t Val = 0;

	if (!PIN_READ(SELECTOR_P_PIN)) {BITSET(Val, 0);}
	if (!PIN_READ(SELECTOR_R_PIN)) {BITSET(Val, 1);}
	if (!PIN_READ(SELECTOR_N_PIN)) {BITSET(Val, 2);}
	if (!PIN_READ(SELECTOR_D_PIN)) {BITSET(Val, 3);}
	if (!PIN_READ(SELECTOR_3_PIN)) {BITSET(Val, 4);}
	if (!PIN_READ(SELECTOR_2_PIN)) {BITSET(Val, 5);}
	if (!PIN_READ(SELECTOR_4_PIN)) {BITSET(Val, 6);}
	if (!PIN_READ(SELECTOR_L_PIN)) {BITSET(Val, 7);}

	return Val;
}

// Текущая позиция селектора АКПП.
void selector_position() {
	// На селекторе АКПП 6 позиций + две дополнительные.
	// Это позволяет использовать один порт контроллера.
	uint8_t Val = get_selector_byte();
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
	static uint8_t Counter = 0;	// Счетчик для задержки отключения флага TCU.EngineWork.

	// Педаль тормоза.
	#ifdef INVERSE_BREAK_PEDAL
		TCU.Break = PIN_READ(BREAK_PEDAL_PIN) ? 0 : 1;
	#else
		TCU.Break = PIN_READ(BREAK_PEDAL_PIN) ? 1 : 0;
	#endif

	// Флаг работы двигателя.
	uint8_t EW = TCU.EngineWork;
	EW = PIN_READ(ENGINE_WORK_PIN) ? 1 : 0;

	if (EW) {	// При положительном сигнале устанавливается флаг и сбрасывается счетчик.
		Counter = 0;
		TCU.EngineWork = EW;
	}
	else {			// При отрицательном сигнале.
		if (TCU.EngineWork) {			// И установленном флаге.
			Counter++;					// Счетчик увеличивается на единицу.
			if (Counter == 3) {			// По достижении порога сбрасывается флаг работы двигателя.
				TCU.EngineWork = EW;
				Counter = 0;				// Сбрасывается счетчик (на всякий случай).
				update_eeprom_adaptation();	// И сохраняются таблицы в EEPROM.
			}
		}
	}
}