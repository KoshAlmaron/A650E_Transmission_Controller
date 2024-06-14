#include <stdint.h>			// Коротние название int.
#include <avr/io.h>			// Названия регистров и номера бит.

#include "tculogic.h"		// Свой заголовок.
#include "configuration.h"	// Настройки.
#include "macros.h"			// Макросы.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "gears.h"			// Фунции переключения передач.

void slt_control() {
	TCU.SLT = get_slt_value();
	OCR1A = TCU.SLT;	// SLT - выход A таймера 1.
}

void at_mode_control() {
	// Если ничего не поменялось, валим.
	if (TCU.ATMode == TCU.Selector) {return;}
	// Если селектор не инициализировался, валим. 
	if (TCU.Selector == 0) {return;}

	// При ошибке селектора, переключаем соленойды в режим первой передачи.
	// При этом можно будет двигаться вперед и назад.
	if (TCU.Selector == 9) {
		TCU.ATMode = TCU.Selector;
		set_gear_n(250);
		set_gear_1(250);
		return;
	}

	// Определение состояния при запуске ЭБУ и при ошибке.
	switch (TCU.ATMode) {
		case 0:
			// В любом случае устанавливаем соленойды в режим N.
			set_gear_n(500);
			// Если селектор в положении P или N.
			if (TCU.Selector == 1 || TCU.Selector == 3) {TCU.ATMode = TCU.Selector;}
			else {TCU.ATMode = 9;}	// Иначе устанавливаем ошибку.
			break;
		case 9:
			// Чтобы убрать ошибку, надо остановиться и включить P или N (1).
			if (TCU.OutputRPM == 0 && (TCU.Selector == 1 || TCU.Selector == 3)) {
				TCU.ATMode = TCU.Selector;
				// Устанавливаем соленойды в режим N.
				set_gear_n(250);
			}
			break;
	}

	// Пока АКПП в начальном состоянии (0) или в ошибке (9) 
	// выходим из функции.
	if (TCU.ATMode == 0 || TCU.ATMode == 9) {return;}


	// Измение состояния АКПП по положению селектора.
	if (TCU.Selector == 1 || TCU.Selector == 3) {	// Нейтраль включается всегда.
		TCU.ATMode = TCU.Selector;
		set_gear_n(0);
		return;
	}

	// Задняя скорость включается только стоя на тормозе.
	if (TCU.Selector == 2) {
		if (TCU.OutputRPM == 0 && TCU.Break) {
			set_gear_n(100);
			set_gear_r(500);
			TCU.ATMode = TCU.Selector;
		}
		return;
	}

	// Включение режимов движения вперед 4-8 (D, D4, 3, L2, L).
	if (TCU.Selector >= 4 && TCU.Selector <= 8) {
		switch (TCU.Gear) {
			case 0:			// С нейтрали.
				TCU.ATMode = TCU.Selector;
				set_gear_1(500);
				break;
			case -1:		// С задней передачи.
				TCU.ATMode = TCU.Selector;
				set_gear_n(100);
				set_gear_1(500);
				break;
			default:		// В остальных случаях просто меняем режим АКПП.
				TCU.ATMode = TCU.Selector;
		}
	}

}

void speedometer_control() {
	OCR3A = TCU.SpdTimerVal;	// Выход на спидометр.
}