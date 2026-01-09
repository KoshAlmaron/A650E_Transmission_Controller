#include <stdint.h>			// Коротние название int.
#include <avr/interrupt.h>	// Прерывания.
#include <avr/io.h>			// Названия регистров и номера бит.
#include <stdio.h>			// Стандартная библиотека ввода/вывода.
#include <avr/wdt.h>		// Сторожевой собак.
#include <util/delay.h>		// Задержки.

#include "macros.h"			// Макросы.
#include "pinout.h"			// Список назначенных выводов.
#include "configuration.h"	// Настройки.

#include "uart.h"			// UART.
#include "i2c.h"			// I2C.
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
#include "buttons.h"		// Кнопки.
#include "bmp180.h"			// Модуль измерения давления.

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
static uint16_t BaroTimer = 0;

uint16_t WaitTimer = 0;		// Таймер ожидания.
uint16_t DebugTimer = 0;	// Таймер блока отладки.

// Прототипы функций.
void loop_main(uint8_t Wait);
static void loop_add();

int main() {
	wdt_enable(WDTO_500MS);	// Сторожевой собак на 500 мс на время инициализации.
	cli();					// Отключаем глобальные прерывания на время инициализации.
		uart_init(3);		// Настройка uart на прием и передачу.
		i2c_init();
		timers_init();		// Настройка таймеров.
		adc_init();			// Настройка АЦП
		selector_init();	// Настройка выводов для селектора.
		solenoid_init();	// Настройка выходов селеноидов, а также лампы заднего хода.
		buttons_init();		// Настройка выводов для типтроника.
		debug_mode_init();	// Настройка перефирии для режима отладки.

		wdt_reset();			// Сброс сторожевого таймера.
		read_eeprom_tables();	// Чтение основных таблиц из EEPROM.
		wdt_reset();
		read_eeprom_adc();		// Чтение таблиц АЦП из EEPROM.
		wdt_reset();
		read_eeprom_speed();	// Чтение таблиц скоростей  переключений из EEPROM.
		wdt_reset();
		read_eeprom_config();	// Чтение параметров из EEPROM.
		wdt_reset();
	sei();				// Включаем глобальные прерывания.

	wdt_enable(WDTO_250MS);	// Сторожевой собак на 250 мс.
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
			BaroTimer += TimerAdd;
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

	if (DataUpdateTimer >= 50) {
		DataUpdateTimer = 0;
		calculate_tcu_data();		// Расчет значений TCU.
		update_gear_speed();		// Обновление порогов переключения передач.
		speedometer_control();		// Выход на спидометр.
	}

	if (TPSTimer >= 75) {
		TPSTimer = 0;
		calc_tps();					// Расчет ДПДЗ с замедлением.
	}

	if (SLTPressureTimer >= 47) {
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
		if(uart_tx_ready()) {
			UartTimer = 0;
			uart_command_processing();
			uart_send_tcu_data();
			TCU.CycleTime = 0;

			if (TCU.AdaptationFlagTPS > 0) {TCU.AdaptationFlagTPS--;}
			else if (TCU.AdaptationFlagTPS < 0) {TCU.AdaptationFlagTPS++;}

			if (TCU.AdaptationFlagTemp > 0) {TCU.AdaptationFlagTemp--;}
			else if (TCU.AdaptationFlagTemp < 0) {TCU.AdaptationFlagTemp++;}
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

	// Барометр.
	if (BaroTimer >= 20) {
		BaroTimer = 0;
		bmp_proccess();
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

	if (ButtonsTimer >= 25) {
		ButtonsTimer = 0;
		buttons_update();	// Обновление состояния кнопок.
	}

	if (AtModeTimer >= 67) {
		AtModeTimer = 0;
		at_mode_control();		// Управление режимами АКПП.
	}

	if (GearsTimer >= 95) {
		GearsTimer = 0;

		gear_control();
		slip_detect();
		buttons_clear();	// Сброс необработанных состояний.
	}

	if (SLUPressureTimer >= 25) {
		SLUPressureTimer = 0;
		slu_gear2_control();		// Управление давлением SLU для второй передачи.
	}
}

// Прерывание при совпадении регистра сравнения OCR0A на таймере 0 каждую 1мс. 
ISR (TIMER0_COMPA_vect) {MainTimer++;}

// Прерывание при совпадении регистра сравнения OCR2A на таймере 2 каждую 10 мкс. 
ISR (TIMER2_COMPA_vect) {CycleTimer++;}

