#include <stdint.h>			// Коротние название int.
#include <avr/interrupt.h>	// Прерывания.
#include <avr/io.h>			// Названия регистров и номера бит.
#include <stdio.h>			// Стандартная библиотека ввода/вывода.
#include <avr/wdt.h>		// Сторожевой собак.
#include <util/delay.h>		// Задержки.

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
#include "lcd.h"			// LCD экран.
#include "buttons.h"			// Кнопки.

// Основной счетчик времени,
// увеличивается по прерыванию на единицу каждую 1 мс.
volatile uint8_t MainTimer = 0;
// Счетчик времени цикла 10 мкс.
volatile uint16_t CycleTimer = 0;

// Счетчики времени.
static uint16_t UartTimer = 0;
static uint16_t SensorTimer = 0;
static uint16_t SelectorTimer = 0;
static uint16_t DataUpdateTimer = 0;
static uint16_t AtModeTimer = 0;
static uint16_t GearsTimer = 0;
static uint16_t TPSTimer = 0;
static uint16_t GlockTimer = 0;
static uint16_t SLTPressureTimer = 0;
static uint16_t SLUPressureTimer = 0;
static uint16_t ButtonsTimer = 0;

// Таймер ожидания.
uint16_t WaitTimer = 0;

// Состояние режима отладки:
// 0 - выкл, 1 - только экран, 2 - экран + ручное управление.
uint16_t DebugTimer = 0;

// Прототипы функций.
void loop_main(uint8_t Wait);
static void loop_add();

int main() {
	wdt_enable(WDTO_500MS);	// Сторожевой собак на 500 мс на время инициализации.
	cli();					// Отключаем глобальные прерывания на время инициализации.
		uart_init(2);		// Настройка uart только на передачу.
		timers_init();		// Настройка таймеров.
		adc_init();			// Настройка АЦП
		selector_init();	// Настройка выводов для селектора.
		solenoid_init();	// Настройка выходов селеноидов, а также лампы заднего хода.
		debug_mode_init();	// Настройка перефирии для режима отладки.
		read_eeprom();		// Чтение параметров из EEPROM.
	sei();				// Включаем глобальные прерывания.

	#ifdef DEBUG_MODE_PRINT
		uart_send_string("Start\n");
	#endif


	//wdt_enable(WDTO_250MS);	// Сторожевой собак на 250 мс.

	SET_PIN_MODE_OUTPUT(CYCLE_TEST_PIN);	// Пин проверки скорости цикла.

	while(1) {
		loop_main(0);			// Основной цикл, выполняется всегда.
		loop_add();				// Вспомогательный цикл.
	}
	return 0;
}

// Основной цикл.
void loop_main(uint8_t Wait) {
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
		DebugTimer += TimerAdd;
		TPSTimer += TimerAdd;
		GlockTimer += TimerAdd;
		SLTPressureTimer += TimerAdd;
		ButtonsTimer += TimerAdd;

		if (WaitTimer > TimerAdd) {WaitTimer -= TimerAdd;}
		else {WaitTimer = 0;}

		// Отключение счетчиков дополнительного цикла.
		if (!Wait) {
			AtModeTimer += TimerAdd;
			GearsTimer += TimerAdd;
			SLUPressureTimer += TimerAdd;
		}
	}

	// Обработка датчиков.
	if (SensorTimer >= 4) {
		SensorTimer = 0;
		adc_read();					// Считывание значений АЦП.
	}

	if (SelectorTimer >= 202) {
		SelectorTimer = 0;
		selector_position();		// Определение позиции селектора АКПП.
		engine_n_break_state();		// Состояние двигателя и педали тормоза.
		rear_lamp();				// Лампа заднего хода.
	}

	if (DataUpdateTimer >= 40) {
		DataUpdateTimer = 0;
		calculate_tcu_data();		// Расчет значений TCU.
		update_gear_speed();		// Обновление порогов переключения передач.
	}

	if (TPSTimer >= 75) {
		TPSTimer = 0;
		calc_tps();					// Расчет ДПДЗ с замедлением.
	}

	if (SLTPressureTimer >= 26) {
		slt_control();				// Управление линейными давлением.
		SLTPressureTimer = 0;
	}	

	if (GlockTimer >= 100) {
		glock_control(GlockTimer);
		GlockTimer = 0;
	}

	// Фиксация максимального времени цикла от предыдущей отправки пакета.
	uint16_t CT = 0;
	cli();
		CT = CycleTimer;
		CycleTimer = 0;
	sei();
	if (CT > TCU.CycleTime) {TCU.CycleTime = CT;}

	// Отправка данных в UART.
	if (UartTimer >= 50) {
		PIN_TOGGLE(CYCLE_TEST_PIN);
		if(uart_tx_ready()) {
			UartTimer = 0;
			#ifdef DEBUG_MODE_PRINT
				uart_send_string("Cycle - ");
				uart_send_uint16(TCU.CycleTime);
				uart_send_char('0');
				uart_send_string(" / D - ");
				uart_send_uint16(TCU.DebugMode);
				uart_send_char('\n');

				cli();
					CycleTimer = 0;
				sei();
			#else
				send_tcu_data();
			#endif
			TCU.CycleTime = 0;
		}
	}
}

static void loop_add() {
	lcd_process_step();
	// Режим отладки.
	if (DebugTimer >= 50) {
		DebugTimer = 0;
		debug_loop();
	}

	if (TCU.DebugMode == 2) {return;}	// Ручное управление соленоидами.

	// При неработающем двигателе выключаем все соленоиды
	if (!TCU.EngineWork) {
		SET_PIN_LOW(SOLENOID_S1_PIN);
		SET_PIN_LOW(SOLENOID_S2_PIN);
		SET_PIN_LOW(SOLENOID_S3_PIN);
		SET_PIN_LOW(SOLENOID_S4_PIN);

		// Устанавливаем ШИМ на соленоидах.
		TCU.SLT = 1023;
		TCU.SLN = 0;
		TCU.SLU = 0;

		cli();
			OCR1A = TCU.SLT;	// SLT (При выключенном соленоиде максимальное давление).
			OCR1B = TCU.SLN;	// SLN.
			OCR1C = TCU.SLU;	// SLU.
		sei();

		TCU.ATMode = 0;	// Состояние АКПП.
		TCU.Gear = 0;
		TCU.Glock = 0;
		return;
	}

	if (ButtonsTimer >= 25) {		// Обработка кнопок.
		ButtonsTimer = 0;
		buttons_update();
	}

	if (AtModeTimer >= 67) {
		AtModeTimer = 0;
		at_mode_control();		// Управление режимами АКПП.
	}

	if (GearsTimer >= 95) {
		GearsTimer = 0;
		gear_control();
		slip_detect();
	}

	if (SLUPressureTimer >= 25) {
		slu_gear2_control(SLUPressureTimer);// Управление давлением SLU для второй передачи.
		SLUPressureTimer = 0;
	}
}

// Прерывание при совпадении регистра сравнения OCR0A на таймере 0 каждую 1мс. 
ISR (TIMER0_COMPA_vect) {MainTimer++;}

// Прерывание при совпадении регистра сравнения OCR2A на таймере 2 каждую 10 мкс. 
ISR (TIMER2_COMPA_vect) {CycleTimer++;}

