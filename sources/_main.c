#include <stdint.h>			// Коротние название int.
#include <avr/interrupt.h>	// Прерывания.
#include <avr/io.h>			// Названия регистров и номера бит.
#include <stdio.h>			// Стандартная библиотека ввода/вывода.
#include <avr/wdt.h>		// Сторожевой собак.

#include "macros.h"			// Макросы.
#include "mathemat.h"		// Математические функции.
#include "pinout.h"			// Список назначенных выводов.
#include "configuration.h"	// Настройки.

#include "uart.h"			// UART.
#include "timers.h"			// Таймеры.
#include "adc.h"			// АЦП.
#include "spdsens.h"		// Датчики скорости валов.
#include "selector.h"		// Положение селектора АКПП.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "eeprom.h"			// Чтение и запись EEPROM.
#include "tculogic.h"		// Управление режимами АКПП.
#include "gears.h"			// Фунции переключения передач.
#include "debug.h"			// Модуль отладки.

// Основной счетчик времени,
// увеличивается по прерыванию на единицу каждую 1мс.
volatile uint8_t MainTimer = 0;

// Счетчики времени.
int16_t UartTimer = 0;
int16_t SensorTimer = 0;
int16_t SelectorTimer = 0;
int16_t DataUpdateTimer = 0;
int16_t AtModeTimer = 0;
int16_t GearsTimer = 0;
int16_t TPSTimer = 0;
int16_t GlockTimer = 0;
int16_t PressureControlTimer = 0;

// Таймер ожидания.
extern int16_t WaitTimer;

// Состояние режима отладки:
// 0 - выкл, 1 - только экран, 2 - экран + ручное управление.
uint8_t DebugMode = 0;
uint16_t DebugTimer = 0;

// Прототипы функций.
void loop_main();
static void loop_add();

int main() {
	wdt_enable(WDTO_250MS);	// Сторожевой собак на 250 мс.
	cli();				// Отключаем глобальные прерывания на время инициализации.
		uart_init(2);		// Настройка uart только на передачу.
		timers_init();		// Настройка таймеров.
		adc_init();			// Настройка АЦП
		selector_init();	// Настройка выводов для селектора.
		solenoid_init();	// Настройка выходов селеноидов, а также лампы заднего хода.
		debug_mode_init();	// Настройка перефирии для режима отладки.
		read_eeprom();		// Чтение параметров из EEPROM.
	sei();				// Включаем глобальные прерывания

	while(1) {
		loop_main();			// Основной цикл, выполняется всегда.
		loop_add();				// Вспомогательный цикл.
	}
	return 0;
}

// Основной цикл.
void loop_main() {
	wdt_reset();			// Сброс сторожевого таймера.
	uint8_t TimerAdd = 0;
	cli();
		TimerAdd = MainTimer;
		MainTimer = 0;
	sei();
	
	// Счетчики времени.
	if (TimerAdd) {
		UartTimer += TimerAdd;
		SensorTimer += TimerAdd;
		SelectorTimer += TimerAdd;
		DataUpdateTimer += TimerAdd;
		AtModeTimer += TimerAdd;
		DebugTimer += TimerAdd;
		WaitTimer += TimerAdd;	// Внешняя переменная из gears.c.
		GearsTimer += TimerAdd;
		TPSTimer += TimerAdd;
		GlockTimer += TimerAdd;
		PressureControlTimer += TimerAdd;
	}

	// Таймер ожидание д.б. <= 0.
	if (WaitTimer > 0) {WaitTimer = 0;}

	// Обработка датчиков.
	if (SensorTimer >= 4) {
		SensorTimer = 0;
		adc_read();					// Считывание значений АЦП.
	}

	if (SelectorTimer >= 202) {
		SelectorTimer = 0;
		selector_position();		// Определение позиции селектора АКПП.
		engine_n_break_state();		// Состояние двинателя и педали тормоза.
		rear_lamp();				// Лампа заднего хода.
	}

	if (DataUpdateTimer >= 20) {
		DataUpdateTimer = 0;
		calculate_tcu_data();		// Расчет значений TCU.
		speedometer_control();		// Выход на спидометр.
		update_gear_speed();		// Обновление порогов переключения передач.
	}

	if (TPSTimer >= 153) {
		TPSTimer = 0;
		calc_tps();					// Расчет ДПДЗ с замедлением.
	}

	if (GlockTimer >= 100) {
		glock_control(GlockTimer);
		GlockTimer = 0;
	}

	// Отправка данных в UART.
	if (UartTimer >= 24) {
		if(uart_tx_ready()) {
			UartTimer = 0;
			send_tcu_data();
		}
	}
}

static void loop_add() {
	// Режим отладки.
	if (DebugTimer >= 50) {
		DebugTimer = 0;
		debug_loop();
	}

	if (DebugMode == 2) {return;}	// Ручное управление соленоидами.

	// При неработающем двигателе выключаем все соленоиды
	if (!TCU.EngineWork) {
		SET_PIN_LOW(SOLENOID_S1_PIN);
		SET_PIN_LOW(SOLENOID_S2_PIN);
		SET_PIN_LOW(SOLENOID_S3_PIN);
		SET_PIN_LOW(SOLENOID_S4_PIN);

		// Устанавливаем ШИМ на соленоидах.
		OCR1A = 255;	// SLT (При выключенном соленоиде максимальное давление).
		OCR1B = 0;		// SLN.
		OCR1C = 0;		// SLU.

		TCU.ATMode = 0;	// Состояние АКПП.
		TCU.Gear = 0;
		TCU.Glock = 0;
		return;
	}

	if (AtModeTimer >= 67) {
		AtModeTimer = 0;
		at_mode_control();		// Управление режимами АКПП.
	}

	if (GearsTimer >= 95) {
		//GearsTimer = -1 * gear_control();
		gear_control();
		slip_detect();
		GearsTimer = 0;
	}

	if (PressureControlTimer >= 97) {
		slt_control();								// Управление линейными давлением.
		slu_gear2_control(PressureControlTimer); 	// Управление давлением SLU для второй передачи.
		PressureControlTimer = 0;
	}

}

// Прерывание при совпадении регистра сравнения OCR0A на таймере 0 каждую 1мс. 
ISR (TIMER0_COMPA_vect) {MainTimer++;}