#include <stdint.h>			// Коротние название int.
#include <avr/io.h>			// Названия регистров и номера бит.

#include "tculogic.h"		// Свой заголовок.
#include "configuration.h"	// Настройки.
#include "macros.h"			// Макросы.
#include "pinout.h"			// Список назначенных выводов.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "gears.h"			// Фунции переключения передач.

// Прототипы локальных функций.
static void engine_brake_solenoid();

// Управление линейным давлением SLT.
void slt_control() {
	TCU.SLT = get_slt_pressure();
	OCR1A = TCU.SLT;	// SLT - выход A таймера 1.
}

// Управление давлением в гидроаккумуляторах SLN.
void sln_control() {
	// Данная функция плавно уменьшает давление после переключения передачи.
	if (TCU.SLN > SLN_MIN_VALUE && TCU.Gear != 0) {
		TCU.SLN -= 1;
		OCR1B = TCU.SLN;	// SLN - выход B таймера 1.	
	}
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
		set_gear_n();
		set_gear_1();
		return;
	}

	// Определение состояния при запуске ЭБУ и при ошибке.
	switch (TCU.ATMode) {
		case 0:
			// В любом случае устанавливаем соленойды в режим N.
			set_gear_n();
			// Если селектор в положении P или N.
			if (TCU.Selector == 1 || TCU.Selector == 3) {TCU.ATMode = TCU.Selector;}
			else {TCU.ATMode = 9;}	// Иначе устанавливаем ошибку.
			break;
		case 9:
			// Чтобы убрать ошибку, надо остановиться и включить P или N (1).
			if (TCU.OutputRPM == 0 && (TCU.Selector == 1 || TCU.Selector == 3)) {
				TCU.ATMode = TCU.Selector;
				// Устанавливаем соленойды в режим N.
				set_gear_n();
			}
			break;
	}

	// Пока АКПП в начальном состоянии (0) или в ошибке (9) 
	// выходим из функции.
	if (TCU.ATMode == 0 || TCU.ATMode == 9) {return;}

	// Измение состояния АКПП по положению селектора.
	if (TCU.Selector == 1 || TCU.Selector == 3) {	// Нейтраль включается всегда.
		TCU.ATMode = TCU.Selector;
		set_gear_n();
		return;
	}

	// Задняя скорость включается только стоя на тормозе.
	if (TCU.Selector == 2) {
		if (TCU.CarSpeed < 5 && TCU.Break) {
			set_gear_n();
			set_gear_r();
			TCU.ATMode = TCU.Selector;
		}
		else {disable_gear_r();}	 // Принудительно выключаем задний ход.
		return;
	}

	// Включение режимов движения вперед 4-8 (D, D4, 3, L2, L).
	if (TCU.Selector >= 4 && TCU.Selector <= 8) {
		switch (TCU.Gear) {
			case 0:			// С нейтрали.
				TCU.ATMode = TCU.Selector;
				set_gear_1();
				break;
			case -1:		// С задней передачи.
				TCU.ATMode = TCU.Selector;
				set_gear_n();
				set_gear_1();
				break;
			default:		// В остальных случаях просто меняем режим АКПП.
				TCU.ATMode = TCU.Selector;
				// Проверка дополнительных соленоидов торможения двигателем.
				engine_brake_solenoid();
		}
	}
}

static void engine_brake_solenoid() {
	switch (TCU.Gear) {
		case 1:
			// Дополнительный соленоид первой передачи.		
			if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}		// 2
			else {SET_PIN_LOW(SOLENOID_S3_PIN);}
			break;
		case 3:
			// Дополнительный соленоид третьей передачи.
			if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}		// D
			else {SET_PIN_LOW(SOLENOID_S3_PIN);}	
			break;
	}
}

void glock_control(uint8_t Timer) {
	static uint16_t GTimer = 0;

	// Условия для включения блокировки гидротрансформатора.
	if (!TCU.Break 
			&& TCU.Gear >= 4 
			&& TCU.TPS >= TPS_IDLE_LIMIT 
			&& TCU.TPS <= GLOCK_MAX_TPS 
			&& TCU.OilTemp >= 50
			&& TCU.CarSpeed >= 40) {
		if (!TCU.Glock) {GTimer += Timer;}
	}
	else {	// Отключение блокировки при нарушении условий.
		if (TCU.Glock) {
			GTimer = 0;
			TCU.Glock = 0;
			TCU.SLU = 0;
			OCR1C = TCU.SLU;	// SLU - выход C таймера 1.
		}
		return;
	}

	// Задержка включения блокировки.
	if (GTimer > 3000) {
		if (!TCU.Glock) {
			TCU.SLU = SLU_GLOCK_START_VALUE;	// Начальное значение схватывания.
			TCU.Glock = 1;
		}
		else {
			if (TCU.SLU >= SLU_GLOCK_MAX_VALUE) {return;}
			uint8_t PressureAdd = 5;
			if (TCU.SLU < 80) {PressureAdd = 2;}
			TCU.SLU = MIN(SLU_GLOCK_MAX_VALUE, TCU.SLU + PressureAdd);
			OCR1C = TCU.SLU;	// Применение значения.
		}
	}
}

void rear_lamp() {
	if (TCU.EngineWork) {
		// При работающем двигателе лампа заднего хода
		// Зависит от текущей передачи.
		if (TCU.Gear == -1) {SET_PIN_HIGH(REAR_LAMP_PIN);}
		else {SET_PIN_LOW(REAR_LAMP_PIN);}
	}
	else {
		// В противном случае лампа включается селектором АКПП.
		if (TCU.Selector == 2) {SET_PIN_HIGH(REAR_LAMP_PIN);}
		else {SET_PIN_LOW(REAR_LAMP_PIN);}
	}
}

void speedometer_control() {
	OCR3A = TCU.SpdTimerVal;	// Выход на спидометр.
}