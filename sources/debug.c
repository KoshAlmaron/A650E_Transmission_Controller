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
#include "eeprom.h"			// Чтение и запись EEPROM.
#include "configuration.h"	// Настройки.

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

static int16_t* ArrayI;
static uint16_t* ArrayU;

#define COLUMN_COUNT 5
#define SCREEN_COUNT 12

extern int8_t MinGear[];
extern int8_t MaxGear[];

// Локальные переменные для хранения последнего значения TPS, 
// это нужно для установки курсора один раз после переключения.
uint8_t LastGearChangeTPS = 0;

// Прототипы функций.
static void lcd_start();
static void print_data();
static void print_dispay_main();

static void print_config_gear2_slu_pressure();
static void print_config_gear2_slu_temp_corr();
static void print_config_gear2_react_add();
static void print_config_gear2_tps_adaptation();
static void print_config_gear2_temp_adaptation();

static void print_config_gear3_slu_pressure();
static void print_config_gear3_slu_delay();
static void print_config_gear3_sln_offset();

static void print_config_sln_pressure();

static void print_config_slt_pressure();
static void print_config_slt_temp_corr();

static void update_gear_ratio();
static void print_values(uint8_t GridType, uint8_t ArrayType, uint8_t Step, int16_t Min, int16_t Max, int16_t Ratio);

static void print_config_d4_max_gear();
static void solenoid_manual_control();

static void button_action();
static void buttons_clear();
static void buttons_update();
static void button_read(uint8_t State, uint8_t N);

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
			buttons_update();
			button_action();
			buttons_clear();

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
			print_config_gear2_slu_pressure();
			break;
		case 2:
			print_config_gear2_slu_temp_corr();
			break;	
		case 3:
			print_config_gear2_react_add();
			break;
		case 4:
			print_config_gear2_tps_adaptation();
			break;
		case 5:
			print_config_gear2_temp_adaptation();
			break;
		case 6:
			print_config_gear3_slu_pressure();
			break;
		case 7:
			print_config_gear3_slu_delay();
			break;
		case 8:
			print_config_gear3_sln_offset();
			break;
		case 9:
			print_config_sln_pressure();
			break;
		case 10:
			print_config_slt_pressure();
			break;
		case 11:
			print_config_slt_temp_corr();
			break;
		case 12:
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

// Экран настройки давления в тормозе B3 для включения второй передачи.
static void print_config_gear2_slu_pressure() {
	//	----------------------
	//	|Gear 2 SLU |100|1.00|
	//	|St 16 P1200 A99 U101|
	//	|  0|  5| 10| 15| 20||
	//	| 67| 72| 74| 77| 81||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "Gear 2 SLU |%3u|%s"
		, TCU.InstTPS
		, GearRatioChar);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "St %2u P%4u A%2u U%3u"
		, MIN(99, TCU.LastStep)
		, TCU.LastPDRTime
		, MIN(99, TCU.GearChangeTPS)
		, MIN(999, TCU.GearChangeSLU));
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayU = SLUGear2Graph;
	print_values(1, 1, 2, 20, 1000, 1);
}

// Экран настройки температурной корекции давления SLU по переключению 1>2.
static void print_config_gear2_slu_temp_corr() {
	//	----------------------
	//	|SLU G2 TCor|100|1.00|
	//	|UP-12 UV 12 A99 U101|
	//	|  0|  5| 10| 15| 20||
	//	| 67| 72| 74| 77| 81||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "SLU G2 TCor|%3u|%s"
		, TCU.InstTPS
		, GearRatioChar);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "UP%3i UV%3i A%2u U%3u"
		, get_slu_gear2_temp_corr(0)					// В %.
		, get_slu_gear2_temp_corr(TCU.GearChangeSLU)	// В единицах ШИМ.
		, MIN(99, TCU.GearChangeTPS)
		, MIN(999, TCU.GearChangeSLU));
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayI = SLUGear2TempCorrGraph;
	print_values(0, 0, 1, -50, 50, 1);
}

static void print_config_gear2_react_add() {
	//	----------------------
	//	|SLU G2r Add|100|1.00|
	//	|UP-12 UV 12 A99 U101|
	//	|  0|  5| 10| 15| 20||
	//	| 67| 72| 74| 77| 81||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "SLU G2r Add|%3u|%s"
		, TCU.InstTPS
		, GearRatioChar);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "UP%3i UV%3i A%2u U%3u"
		, get_slu_gear2_temp_corr(0)					// В %.
		, get_slu_gear2_temp_corr(TCU.GearChangeSLU)	// В единицах ШИМ.
		, MIN(99, TCU.GearChangeTPS)
		, TCU.GearChangeSLU);
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayI = SLUGear2AddGraph;
	print_values(1, 0, 2, -32, 40, 1);
}

// Экран значений адаптации второй передачи по ДПДЗ.
static void print_config_gear2_tps_adaptation() {
	//	----------------------
	//	|Gear 2 ADP |100|1.00|
	//	|UP-12 UV 12 A99 U101|
	//	|  0|  5| 10| 15| 20||
	//	| 67| 72| 74| 77| 81||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "Gear 2 ADP |%3u|%s"
		, TCU.InstTPS
		, GearRatioChar);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "UP%3i UV%3i A%2u U%3u"
		, get_slu_gear2_temp_corr(0)					// В %.
		, get_slu_gear2_temp_corr(TCU.GearChangeSLU)	// В единицах ШИМ.
		, MIN(99, TCU.GearChangeTPS)
		, MIN(999, TCU.GearChangeSLU));
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayI = SLUGear2TPSAdaptGraph;
	print_values(1, 0, 2, -40, 40, 1);
}

// Экран значений адаптации второй передачи по температуре.
static void print_config_gear2_temp_adaptation() {
	//	----------------------
	//	|SLU G2 TADP|100|1.00|
	//	|UP-12 UV 12 A99 U101|
	//	|  0|  5| 10| 15| 20||
	//	| -1|  0|  2|  0| -2||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "SLU G2 TADP|%3u|%s"
		, TCU.InstTPS
		, GearRatioChar);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "UP%3i UV%3i A%2u U%3u"
		, get_slu_gear2_temp_corr(0)					// В %.
		, get_slu_gear2_temp_corr(TCU.GearChangeSLU)	// В единицах ШИМ.
		, TCU.GearChangeTPS
		, MIN(999, TCU.GearChangeSLU));
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayI = SLUGear2TempAdaptGraph;
	print_values(0, 0, 1, -10, 10, 1);
}

// Экран настройки давления в тормозе B2 для включения третьей передачи.
static void print_config_gear3_slu_pressure() {
	//	----------------------
	//	|Gear 3 SLU |100|1.00|
	//	|St 16 P1200 A99 U101|
	//	|  0|  5| 10| 15| 20||
	//	| 67| 72| 74| 77| 81||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "Gear 3 SLU |%3u|%s"
		, TCU.InstTPS
		, GearRatioChar);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "St %2u P%4u A%2u U%3u"
		, MIN(99, TCU.LastStep)
		, TCU.LastPDRTime
		, MIN(99, TCU.GearChangeTPS)
		, MIN(999, TCU.GearChangeSLU));
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayU = SLUGear3Graph;
	print_values(1, 1, 4, 20, 1000, 1);
}

// Экран настройки задержки выключения SLU при включении третьей передачи.
static void print_config_gear3_slu_delay() {
	//	----------------------
	//	|G3 SLU DL x100  |100|
	//	|St 12 P1000 A99 U101|
	//	|  0|  5| 10| 15| 20||
	//	| 67| 72| 74| 77| 81||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "G3 SLU DL x100  |%3u", TCU.InstTPS);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "St %2u P%4u A%2u U%3u"
		, TCU.LastStep
		, TCU.LastPDRTime
		, TCU.GearChangeTPS
		, MIN(999, TCU.GearChangeSLU));
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayU = SLUGear3DelayGraph;
	print_values(1, 1, 100, 0, 1500, 100);
}

// Экран настройки задержки выключения SLN при включении третьей передачи.
static void print_config_gear3_sln_offset() {
	//	----------------------
	//	|G3 SLN Ofs x100 |100|
	//	|St 16 P1200 A99 O101|
	//	|  0|  5| 10| 15| 20||
	//	| 67| 72| 74| 77| 81||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "G3 SLN Ofs x100 |%3u", TCU.InstTPS);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "St %2u P%4u A%2u O%3i"
		, TCU.LastStep
		, TCU.LastPDRTime
		, TCU.GearChangeTPS
		, MIN(999, TCU.GearChangeSLU));
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayI = SLNGear3OffsetGraph;
	print_values(1, 0, 100, -1500, 1500, 100);
}

// Экран настройка давления аккумуляторов SLN.
static void print_config_sln_pressure() {
	//	----------------------
	//	|SLN Press. |100|1.00|
	//	|TP-12 TV 12 A99 N101|
	//	|  0|  5| 10| 15| 20||
	//	| 67| 72| 74| 77| 81||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "SLN Press. |%3u|%s"
		, TCU.InstTPS
		, GearRatioChar);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "TP%3i TV%3i A%2u T%3u"
		, get_slt_temp_corr(0)					// В %.
		, get_slt_temp_corr(TCU.GearChangeSLT)	// В единицах ШИМ.
		, MIN(99, TCU.GearChangeTPS)
		, MIN(999, TCU.GearChangeSLN));
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayU = SLNGraph;
	print_values(1, 1, 4, 20, 1000, 1);
}

// Экран настройки линейного давления SLT по переключению 3>4.
static void print_config_slt_pressure() {
	//	----------------------
	//	|SLT Press. |100|1.00|
	//	|TP-12 TV 12 A99 T101|
	//	|  0|  5| 10| 15| 20||
	//	| 67| 72| 74| 77| 81||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "SLT Press. |%3u|%s"
		, TCU.InstTPS
		, GearRatioChar);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "TP%3i TV%3i A%2u T%3u"
		, get_slt_temp_corr(0)					// В %.
		, get_slt_temp_corr(TCU.GearChangeSLT)	// В единицах ШИМ.
		, MIN(99, TCU.GearChangeTPS)
		, MIN(999, TCU.GearChangeSLT));
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayU = SLTGraph;
	print_values(1, 1, 4, 20, 1000, 1);
}

// Экран настройки температурной корекции давления SLT по переключению 3>4.
static void print_config_slt_temp_corr() {
	//	----------------------
	//	|SLT Tmp Cor|100|1.00|
	//	|TP-12 TV 12 A99 U101|
	//	|  0|  5| 10| 15| 20||
	//	| 67| 72| 74| 77| 81||
	//	----------------------

	snprintf(StringArray, STR_ARR_SZ, "SLT Tmp Cor|%3u|%s"
		, TCU.InstTPS
		, GearRatioChar);
	lcd_update_buffer(0, StringArray);

	snprintf(StringArray, STR_ARR_SZ, "TP%3i TV%3i A%2u T%3u"
		, get_slt_temp_corr(0)					// В %.
		, get_slt_temp_corr(TCU.SLT)	// В единицах ШИМ.
		, MIN(99, TCU.GearChangeTPS)
		, MIN(999, TCU.GearChangeSLT));
	lcd_update_buffer(1, StringArray);

	// Изменяемые значения.
	ArrayI = SLTTempCorrGraph;
	print_values(0, 0, 1, -60, 60, 1);
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
		, MinGear[5]
		, MaxGear[5]);
	lcd_update_buffer(2, StringArray);

	for (uint8_t i = 0; i < 20; i++) {StringArray[i] = ' ';}
	lcd_update_buffer(1, StringArray);
	
	if (CursorPos) {StringArray[7] = 'X';}
	else {StringArray[11] = 'X';}
	lcd_update_buffer(3, StringArray);

	if (CursorPos > 1) {CursorPos = 0;}

	if (ValueDelta) {
		if (!CursorPos) {MaxGear[5] = CONSTRAIN(MaxGear[5] + ValueDelta, 1, 4);}
		else {MinGear[5] = CONSTRAIN(MinGear[5] + ValueDelta, 1, 4);}
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

// GridType		0 - TempGrid, 1 - TPSGrid.
// ArrayType 	0 - int16_t, 1 - uint16_t.
static void print_values(uint8_t GridType, uint8_t ArrayType, uint8_t Step, int16_t Min, int16_t Max, int16_t Ratio) {
	// Находим по сетке позицию курсора.
	if (TCU.GearChangeTPS != LastGearChangeTPS) {
		LastGearChangeTPS = TCU.GearChangeTPS;

		if (GridType) {CursorPos = get_tps_index(TCU.GearChangeTPS);}
		else {CursorPos = get_temp_index(TCU.OilTemp);}
	}

	// Ограничение по длине массива.
	if (GridType && CursorPos >= TPS_GRID_SIZE) {CursorPos = TPS_GRID_SIZE - 1;}
	if (!GridType && CursorPos >= TEMP_GRID_SIZE) {CursorPos = TEMP_GRID_SIZE - 1;}

	if (CursorPos < StartCol) {StartCol = CursorPos;}
	if (CursorPos > StartCol + COLUMN_COUNT - 1) {StartCol = CursorPos + 1 - COLUMN_COUNT;}

	char TMP[4];

	// Изменяемые значения.
	for (uint8_t i = 0; i < COLUMN_COUNT; i++) {
		if (GridType) {snprintf(TMP, 4, "%3u", TPSGrid[StartCol + i]);}
		else {snprintf(TMP, 4, "%3i", TempGrid[StartCol + i]);}
		
		for (uint8_t k = 0; k < 4; k++) {StringArray[i * 4 + k] = TMP[k];}
		StringArray[i * 4 + 3] = '|';
		lcd_update_buffer(2, StringArray);
	}
	for (uint8_t i = 0; i < COLUMN_COUNT; i++) {
		if (ArrayType) {snprintf(TMP, 4, "%3u", MIN(999, ArrayU[StartCol + i] / Ratio));}
		else {snprintf(TMP, 4, "%3i", CONSTRAIN(ArrayI[StartCol + i] / Ratio, -99, 999));}

		for (uint8_t k = 0; k < 4; k++) {StringArray[i * 4 + k] = TMP[k];}
		if (CursorPos == StartCol + i) {
			if (ValueDelta) {
				if (ArrayType) {ArrayU[CursorPos] = CONSTRAIN((int16_t) ArrayU[CursorPos] + ValueDelta * Step, Min, Max);}
				else {ArrayI[CursorPos] = CONSTRAIN((int16_t) ArrayI[CursorPos] + ValueDelta * Step, Min, Max);}
				ValueDelta = 0;
			}
			StringArray[i * 4 + 3] = '<';
		}
		else {StringArray[i * 4 + 3] = '|';}
		lcd_update_buffer(3, StringArray);
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

static void button_action() {
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
		if (ScreenMode > SCREEN_COUNT) {
			ScreenMode = 0;		// Сброс экрана.
			update_eeprom();	// Запись в EEPROM.
		}
	}
}

static void buttons_clear() {
	if (PIN_READ(DEBUG_S1_PIN) && ButtonState[0] > 200) {ButtonState[0] = 100;}
	if (PIN_READ(DEBUG_S2_PIN) && ButtonState[1] > 200) {ButtonState[1] = 100;}
	if (PIN_READ(DEBUG_S3_PIN) && ButtonState[2] > 200) {ButtonState[2] = 100;}
	if (PIN_READ(DEBUG_S4_PIN) && ButtonState[3] > 200) {ButtonState[3] = 100;}

	if (PIN_READ(DEBUG_SCREEN_BUTTON_F_PIN) && ButtonState[4] > 200) {ButtonState[4] = 100;}
	if (PIN_READ(DEBUG_SCREEN_BUTTON_R_PIN) && ButtonState[5] > 200) {ButtonState[5] = 100;}
}

// Вызов каждые 50 мс.
static void buttons_update() {
	button_read(0, PIN_READ(DEBUG_S1_PIN));
	button_read(1, PIN_READ(DEBUG_S2_PIN));
	button_read(2, PIN_READ(DEBUG_S3_PIN));
	button_read(3, PIN_READ(DEBUG_S4_PIN));

	button_read(4, PIN_READ(DEBUG_SCREEN_BUTTON_F_PIN));
	button_read(5, PIN_READ(DEBUG_SCREEN_BUTTON_R_PIN));
}

static void button_read(uint8_t N, uint8_t State) {
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
