#include <stdint.h>			// Коротние название int.
#include <avr/io.h>			// Названия регистров и номера бит.
#include <avr/interrupt.h>	// Прерывания.

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
	if (TCU.DebugMode == 2) {return;}
	TCU.SLT = get_slt_pressure();
	cli();
		OCR1A = TCU.SLT;	// SLT - выход A таймера 1.
	sei();
}

void at_mode_control() {
	// Если ничего не поменялось, валим.
	if (TCU.ATMode == TCU.Selector) {return;}
	// Если селектор не инициализировался, валим. 
	if (TCU.Selector == 0) {return;}

	// При ошибке селектора, устанавливаем соленойды в режим третьей передачи.
	// При этом можно будет двигаться вперед и назад.
	if (TCU.Selector == 9) {
		TCU.ATMode = TCU.Selector;
		SET_PIN_LOW(SOLENOID_S1_PIN);
		SET_PIN_HIGH(SOLENOID_S2_PIN);
		SET_PIN_LOW(SOLENOID_S3_PIN);
		SET_PIN_LOW(SOLENOID_S4_PIN);
		return;
	}

	// Определение состояния при запуске ЭБУ и при ошибке.
	switch (TCU.ATMode) {
		case 0:
			// Если селектор в положении P или N.
			if (TCU.Selector == 1 || TCU.Selector == 3) {TCU.ATMode = TCU.Selector;}
			else {TCU.ATMode = 9;}	// Иначе устанавливаем ошибку.
			// В любом случае устанавливаем соленойды в режим N.
			set_gear_n();
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

	
	if (TCU.Selector == 2) {
		// Задняя скорость включается только стоя на тормозе,
		// или без тормоза, но с режима P.
		if (TCU.CarSpeed < 5 && (TCU.Break || TCU.ATMode == 1)) {
			TCU.ATMode = TCU.Selector;
			set_gear_r();
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
	static uint16_t SLUStartValue = 0;

	// Сброс счетчика блокировки.
	if (TCU.Glock == 64) {
		TCU.Glock  = 0;
		GTimer = 0;
	}

	// Условия для включения блокировки гидротрансформатора.
	if (!TCU.Break 
			&& TCU.Gear >= 4
			//&& !TCU.GearChange
			&& TCU.TPS >= TPS_IDLE_LIMIT 
			&& TCU.TPS <= GLOCK_MAX_TPS 
			&& ((TCU.OilTemp >= 31 && !TCU.Glock) || (TCU.OilTemp >= 30 && TCU.Glock))
			&& TCU.CarSpeed >= 40) {
				if(!TCU.Glock) {GTimer += Timer;}
	}
	else {	// Отключение блокировки при нарушении условий.
		if (TCU.Glock) {
			// При отпускании педали газа сразу отключаем блокировку ГТ.
			if (TCU.TPS < TPS_IDLE_LIMIT) {
				TCU.SLU = SLU_MIN_VALUE;
				cli();
					OCR1C = TCU.SLU;
				sei();
				TCU.Glock = 0;
				GTimer = 0;
				return;
			}

			// Начальное значение схватывания с учетом температурной коррекции.
			SLUStartValue = SLU_GLOCK_START_VALUE + get_slu_gear2_temp_corr(SLU_GLOCK_START_VALUE);
			
			if (TCU.SLU > SLUStartValue + 20) {
				// Устанавливаем давление схватывания + 20.
				TCU.SLU = SLUStartValue + 20;
			}
			else if (TCU.SLU > SLUStartValue - 20) {
				// Плавно снижаем на 10 единиц.
				TCU.SLU -= 8;
			}
			else {
				// Потом выключаем полностью.
				TCU.Glock = 0;
				TCU.SLU = SLU_MIN_VALUE;
			}
			cli();
				OCR1C = TCU.SLU;	// Применение значения.
			sei();
		}
		GTimer = 0;
		return;
	}

	// Задержка включения блокировки.
	if (GTimer > 3000) {
		if (!TCU.Glock) {
			// Начальное значение схватывания с учетом температурной коррекции.
			SLUStartValue = SLU_GLOCK_START_VALUE + get_slu_gear2_temp_corr(SLU_GLOCK_START_VALUE);
			TCU.SLU = SLUStartValue;
			TCU.Glock = 1;
		}
		else {
			if (TCU.SLU >= SLU_GLOCK_MAX_VALUE) {return;}
			uint8_t PressureAdd = 20;
			if (TCU.SLU < SLUStartValue + 60) {PressureAdd = 4;}
			TCU.SLU = MIN(SLU_GLOCK_MAX_VALUE, TCU.SLU + PressureAdd);
		}
		cli();
			OCR1C = TCU.SLU;	// Применение значения.
		sei();
	}
}

void slip_detect() {
	// Не проверять при смене передачи и на малом газу.
	if (TCU.GearChange || TCU.InstTPS < 6) {
		TCU.SlipDetected = 0;
		return;
	}

	if (ABS(rpm_delta(TCU.Gear)) > MAX_SLIP_RPM) {TCU.SlipDetected = 1;}
	else {TCU.SlipDetected = 0;}
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