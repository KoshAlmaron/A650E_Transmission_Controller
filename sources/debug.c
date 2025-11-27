#include <stdint.h>			// Коротние название int.
#include <avr/io.h>			// Названия регистров и номера бит.
#include <stdio.h>			// Стандартная библиотека ввода/вывода
#include <stdlib.h> 		// Общие утилиты.
#include <avr/interrupt.h>	// Прерывания.

#include "debug.h"			// Свой заголовок.
#include "macros.h"			// Макросы.
#include "pinout.h"			// Список назначенных выводов.
#include "lcd.h"			// LCD экран.
#include "i2c.h"			// I2C (TWI).
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "adc.h"			// АЦП.
#include "gears.h"			// Фунции переключения передач.
#include "configuration.h"	// Настройки.

// Состояние режима отладки:
// 0 - выкл, 1 - только экран, 2 - экран + ручное управление.

// Номер экрана для отображения:
static uint8_t ScreenMode = 0;

#define STR_ARR_SZ 25
static char StringArray[STR_ARR_SZ] = {0};		// Массив формирования строки.
static char GearRatioChar[5] = {0};				// Передаточное число.
// Обозначение режимов на экране.
static int8_t ATModeChar[] = {'I', 'P', 'R', 'N', 'D', '4', '3', '2', 'L', 'E', 'M'};

/*
	Состояние кнопок:
		0 - не нажата,
		1..99 - ожидание после нажатия,
		100 - событие обработано,
		101..149 - откат после срабатывания,
		201 - короткое нажатие,
		202 - длинное нажатие.
*/
static uint8_t ButtonState[6] = {0};	// Ввер/внизх, вправо/влево, смена экрана вперёд/назад.
static uint8_t CursorPos = 0;			// Позиция курсора на экране.
static uint8_t StartCol = 0;			// Начальная позиция данных
static int8_t ValueDelta = 0;			// Флаг изменения значения.

#define SCREEN_COUNT 2

// Прототипы функций.
static void lcd_start();
static void print_data();
static void print_dispay_main();

static void update_gear_ratio();

static void print_config_d4_max_gear();
static void solenoid_manual_control();

static void debug_buttons_action();
static void debug_buttons_clear();
static void debug_buttons_update();
static void debug_button_read(uint8_t State, uint8_t N);

// Настройка портов для режима отладки.
void debug_mode_init() {
	// Вход c подтяжкой для переключателей.
	SET_PIN_MODE_INPUT(DEBUG_LCD_ON_PIN);
	SET_PIN_MODE_INPUT(DEBUG_MODE_ON_PIN);
	SET_PIN_HIGH(DEBUG_LCD_ON_PIN);
	SET_PIN_HIGH(DEBUG_MODE_ON_PIN);
	
	SET_PIN_MODE_INPUT(DEBUG_S1_PIN);
	SET_PIN_MODE_INPUT(DEBUG_S2_PIN);
	SET_PIN_MODE_INPUT(DEBUG_S3_PIN);
	SET_PIN_MODE_INPUT(DEBUG_S4_PIN);
	SET_PIN_HIGH(DEBUG_S1_PIN);
	SET_PIN_HIGH(DEBUG_S2_PIN);
	SET_PIN_HIGH(DEBUG_S3_PIN);
	SET_PIN_HIGH(DEBUG_S4_PIN);

	i2c_init();		// Настройка интерфейса I2C (TWI).
}

void debug_loop() {
	// Переключение режимов.
	switch (TCU.DebugMode)	{
		case 0:
			if (!PIN_READ(DEBUG_LCD_ON_PIN)) {
				TCU.DebugMode = 1;
				lcd_start();
			}
			break;
		case 1:
			debug_buttons_update();
			debug_buttons_action();
			debug_buttons_clear();

			print_data();
			if (!PIN_READ(DEBUG_MODE_ON_PIN)) {
				TCU.DebugMode = 2;			// Ручной режим управления.
				ScreenMode = 0;
				add_channels_on(1);	// Увeличить количество каналов ADC.
				TCU.ATMode = 0;
				TCU.Gear = 0;
			}
			if (PIN_READ(DEBUG_LCD_ON_PIN)) {
				TCU.DebugMode = 0;
				add_channels_on(0);	// Уменьшить количество каналов ADC.
			}
			break;
		case 2:
			if (PIN_READ(DEBUG_MODE_ON_PIN)) {
				TCU.DebugMode = 1;
				add_channels_on(0);	// Уменьшить количество каналов ADC.
			}
			if (PIN_READ(DEBUG_LCD_ON_PIN)) {
				TCU.DebugMode = 0;
				add_channels_on(0);	// Уменьшить количество каналов ADC.
			}

			TCU.EngineWork = 1;
			solenoid_manual_control();	// Ручное управление соленоидами.
			print_data();				// Отправка данных на дисплей.
			break;
	}
}

static void lcd_start() {
	lcd_init(0x3f);
}

static void print_data() {
	static uint8_t Counter = 0;
	Counter++;
	if (Counter < 4) {return;}
	if (!lcd_is_ready()) {return;}
	Counter = 0;

	update_gear_ratio();

	switch (ScreenMode)	{
		case 0:
			print_dispay_main();
			break;
		case 1:
			print_config_d4_max_gear();
			break;
	}
	lcd_send_buffer();
	ValueDelta = 0;
}

// Основной экран.
static void print_dispay_main() {
	//	----------------------
	//	|O 070| 00 01 |I 1000|
	//	|T 120| A 110 |O 0900|
	//	|N 120| S D-D |Sp 060|
	//	|U 120| G 4 0 |P 1000|
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "O %3i| %1u%1u %1u%1u |I %4u"
		, TCU.OilTemp
		, MIN(1, TCU.S1)
		, MIN(1, TCU.S2)
		, MIN(1, TCU.S3)
		, MIN(1, TCU.S4)
		, TCU.DrumRPM);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "T%4u| A %3u |O %4u"
		, TCU.SLT
		, TCU.InstTPS
		, TCU.OutputRPM);
	lcd_update_buffer(1, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "N%4u| S %c-%c |Sp %3i"
		, TCU.SLN
		, ATModeChar[TCU.Selector]
		, ATModeChar[TCU.ATMode]
		, TCU.CarSpeed);
	lcd_update_buffer(2, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "U%4u| G%2i %-2u|P %4u"
		, TCU.SLU
		, TCU.Gear
		, MIN(99, TCU.LastStep)
		, TCU.LastPDRTime);
	lcd_update_buffer(3, StringArray);
}

// Временная установка максимальной передачи в режиме D4.
static void print_config_d4_max_gear() {
	//	----------------------
	//	|  D4 Min/Max Gear   |
	//	|                    |
	//	|       4 - 4        |
	//	|                    |
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "  D4 Min/Max Gear   ");
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "       %1u - %1u        "
		, get_min_gear(5)
		, get_max_gear(5));
	lcd_update_buffer(2, StringArray);

	for (uint8_t i = 0; i < 20; i++) {StringArray[i] = ' ';}
	lcd_update_buffer(1, StringArray);
	
	if (CursorPos) {StringArray[7] = 'X';}
	else {StringArray[11] = 'X';}
	lcd_update_buffer(3, StringArray);

	if (CursorPos > 1) {CursorPos = 0;}

	if (ValueDelta) {
		uint8_t Min = get_min_gear(5);
		uint8_t Max = get_max_gear(5);
		if (!CursorPos) {Max = CONSTRAIN(Max + ValueDelta, 1, 5);}
		else {Min = CONSTRAIN(Min + ValueDelta, 1, 5);}

		set_gear_limit(Min, Max);
		ValueDelta = 0;
	}
}

static void update_gear_ratio() {
	if (TCU.OutputRPM > 100) {
		snprintf(GearRatioChar, 5, "%1u.%02u",
			MIN(9, TCU.DrumRPM / TCU.OutputRPM), MIN(99, ((TCU.DrumRPM % TCU.OutputRPM) * 100) / TCU.OutputRPM));
	}
	else {
		GearRatioChar[0] = '-';
		GearRatioChar[1] = '.';
		GearRatioChar[2] = '-';
		GearRatioChar[3] = '-';
		GearRatioChar[4] = ' ';
	}
}

static void solenoid_manual_control() {
	if (PIN_READ(DEBUG_S1_PIN)) {SET_PIN_LOW(SOLENOID_S1_PIN);}
	else {SET_PIN_HIGH(SOLENOID_S1_PIN);}

	if (PIN_READ(DEBUG_S2_PIN)) {SET_PIN_LOW(SOLENOID_S2_PIN);}
	else {SET_PIN_HIGH(SOLENOID_S2_PIN);}

	if (PIN_READ(DEBUG_S3_PIN)) {SET_PIN_LOW(SOLENOID_S3_PIN);}
	else {SET_PIN_HIGH(SOLENOID_S3_PIN);}
	
	if (PIN_READ(DEBUG_S4_PIN)) {SET_PIN_LOW(SOLENOID_S4_PIN);}
	else {SET_PIN_HIGH(SOLENOID_S4_PIN);}


	// Считываем положение потенциометров.
	TCU.SLT = 1023 - get_adc_value(2);
	if (TCU.SLT <= 8) {TCU.SLT = 0;}
	if (TCU.SLT >= 1010) {TCU.SLT = 1023;}

	TCU.SLN = get_adc_value(3);
	if (TCU.SLN <= 8) {TCU.SLN = 0;}
	if (TCU.SLN >= 1010) {TCU.SLN = 1023;}

	TCU.SLU = 200 + (get_adc_value(4) >> 1);

	// Устанавливаем ШИМ на соленоидах.
	cli();
		OCR1A = TCU.SLT;
		OCR1B = TCU.SLN;
		OCR1C = TCU.SLU;
	sei();
}

static void debug_buttons_action() {
	if (ButtonState[0] == 201) {ValueDelta = 1;}	// Короткое S1 Значение +.
	if (ButtonState[0] == 202) {ValueDelta = 2;}	// Длинное S1 Значение ++.

	if (ButtonState[1] == 201) {ValueDelta = -1;}	// Короткое S2 Значение -.
	if (ButtonState[1] == 202) {ValueDelta = -2;}	// Длинное S2 Значение --.

	if (ButtonState[2] == 201) {	// S3 Перемещение курсора -.
		if (CursorPos) {CursorPos -= 1;}
	}
	if (ButtonState[2] == 202) {	// S3 Перемещение курсора --.
		if (CursorPos) {CursorPos -= 1;}
	}

	if (ButtonState[3] == 201) {	// S4 Перемещение курсора +.
		CursorPos += 1;
		if (CursorPos == 255) {CursorPos = 0;}
	}
	if (ButtonState[3] == 202) {	// S4 Перемещение курсора ++.
		CursorPos += 1;
		if (CursorPos == 255) {CursorPos = 0;}
	}

	if (ButtonState[4] == 201) {	// SLU Смена экрана назад.
		CursorPos = 0;
		StartCol = 0;
		if (ScreenMode) {ScreenMode -= 1;}
	}	

	if (ButtonState[5] == 201) {	// SLU Смена экрана вперёд.
		CursorPos = 0;
		StartCol = 0;
		ScreenMode += 1;
		if (ScreenMode > SCREEN_COUNT) {ScreenMode = 0;} // Сброс экрана.
	}
}

static void debug_buttons_clear() {
	if (PIN_READ(DEBUG_S1_PIN) && ButtonState[0] > 200) {ButtonState[0] = 100;}
	if (PIN_READ(DEBUG_S2_PIN) && ButtonState[1] > 200) {ButtonState[1] = 100;}
	if (PIN_READ(DEBUG_S3_PIN) && ButtonState[2] > 200) {ButtonState[2] = 100;}
	if (PIN_READ(DEBUG_S4_PIN) && ButtonState[3] > 200) {ButtonState[3] = 100;}

	if (PIN_READ(DEBUG_SCREEN_BUTTON_F_PIN) && ButtonState[4] > 200) {ButtonState[4] = 100;}
	if (PIN_READ(DEBUG_SCREEN_BUTTON_R_PIN) && ButtonState[5] > 200) {ButtonState[5] = 100;}
}

// Вызов каждые 50 мс.
static void debug_buttons_update() {
	debug_button_read(0, PIN_READ(DEBUG_S1_PIN));
	debug_button_read(1, PIN_READ(DEBUG_S2_PIN));
	debug_button_read(2, PIN_READ(DEBUG_S3_PIN));
	debug_button_read(3, PIN_READ(DEBUG_S4_PIN));

	debug_button_read(4, PIN_READ(DEBUG_SCREEN_BUTTON_F_PIN));
	debug_button_read(5, PIN_READ(DEBUG_SCREEN_BUTTON_R_PIN));
}

static void debug_button_read(uint8_t N, uint8_t State) {
	if (ButtonState[N] < 100) {
		if (!State) {
			ButtonState[N]++;
			if (ButtonState[N] >= 10) {ButtonState[N] = 202;}	// Длиное нажатие.
		}
		else {
			if (ButtonState[N] >= 1) {ButtonState[N] = 201;}	// Короткое нажатие.
		}
		return;
	}

	if (ButtonState[N] < 150 && State) {
		ButtonState[N]++;
		if (ButtonState[N] >= 103) {ButtonState[N] = 0;}	// Сброс состояния кнопки.
		return;		
	}
}
