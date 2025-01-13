#include <stdint.h>			// Коротние название int.
#include <avr/io.h>			// Названия регистров и номера бит.
#include <stdio.h>			// Стандартная библиотека ввода/вывода
#include <stdlib.h> 		// Общие утилиты.

#include "debug.h"			// Свой заголовок.
#include "macros.h"			// Макросы.
#include "pinout.h"			// Список назначенных выводов.
#include "lcd.h"			// LCD экран.
#include "i2c.h"			// I2C (TWI).
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "adc.h"			// АЦП.
#include "eeprom.h"			// Чтение и запись EEPROM.

extern uint8_t DebugMode;	// Переменная из main.

// Номер экрана для отображения:
uint8_t ScreenMode = 0;

char LCDArray[21] = {0};	// Массив для отправки на дисплей.

// Обозначение режимов на экране.
int8_t ATModeChar[] = {'I', 'P', 'R', 'N', 'D', '4', '3', '2', 'L', 'E', 'M'};

/*
	Состояние кнопок:
		0 - не нажата,
		1..99 - ожидание после нажатия,
		100 - событие обработано,
		101..149 - откат после срабатывания,
		201 - короткое нажатие,
		202 - длинное нажатие.
*/
uint8_t ButtonState[5] = {0};	// Ввер/внизх, вправо/влево, смена экрана.
uint8_t CursorPos = 0;			// Позици курсора на экране.
uint8_t StartCol = 0;			// Начальная позиция данных
int8_t ValueDelta = 0;			// Флаг изменения значения.

#define COLUMN_COUNT 5
#define SCREEN_COUNT 8

extern uint8_t LastGear2ChangeTPS;			// Значение ДПДЗ при последнем переключении 1>2.
extern uint8_t LastGear2ChangeSLU;			// Значение SLU при последнем переключении 1>2.

extern uint8_t LastGear2ReactivateTPS;		// Значение ДПДЗ при возобновлении второй передачи.
extern uint8_t LastGear2ReactivateRPM;		// Значение опережения при возобновлении второй передачи.

extern uint8_t LastGear3ChangeTPS;			// Значение ДПДЗ при последнем переключении 2>3.
extern uint16_t LastGear3ChangeSLU;			// Задержка отключения SLU при последнем переключении 2>3.

extern uint8_t LastGear4ChangeTPS;			// Значение ДПДЗ при последнем переключении 3>4.
extern uint8_t LastGear4ChangeSLT;			// Значение SLT при последнем переключении 3>4.
extern uint8_t LastGear4ChangeSLN;			// Значение SLN при последнем переключении 3>4.

extern int8_t MaxGear[];

// Локальные значения для хранения, так как переменные из gears.c будут обнуляться 
// после считывания значений.
uint8_t Gear2ChangeTPS = 0;
uint8_t Gear2ChangeSLU = 0;

uint8_t Gear2ReactivateTPS = 0;
uint8_t Gear2ReactivateRPM = 0;

uint8_t Gear3ChangeTPS = 0;
uint16_t Gear3ChangeSLU = 0;

uint8_t Gear4ChangeTPS = 0;
uint8_t Gear4ChangeSLT = 0;
uint8_t Gear4ChangeSLN = 0;

// Прототипы функций.
static void lcd_start();
static void print_data();
static void print_dispay_main();

static void print_config_slt_pressure();
static void print_config_slt_temp_corr();
static void print_config_sln_pressure();
static void print_config_gear2_slu_pressure();
static void print_config_gear2_slu_temp_corr();
static void print_config_gear3_slu_pressure();
static void print_config_gear2_reactivate();
static void print_config_d4_max_gear();

static uint8_t get_tps_index(uint8_t TPS);
static uint8_t get_temp_index(int16_t Temp);

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
	switch (DebugMode)	{
		case 0:
			if (!PIN_READ(DEBUG_LCD_ON_PIN)) {
				DebugMode = 1;
				lcd_start();
			}
			break;
		case 1:
			buttons_update();
			button_action();
			buttons_clear();

			print_data();
			if (!PIN_READ(DEBUG_MODE_ON_PIN)) {
				DebugMode = 2;			// Ручной режим управления.
				ScreenMode = 0;
				add_channels_on(1);	// Увeличить количество каналов ADC.
				TCU.ATMode = 0;
				TCU.Gear = 0;
			}
			if (PIN_READ(DEBUG_LCD_ON_PIN)) {
				DebugMode = 0;
				add_channels_on(0);	// Уменьшить количество каналов ADC.
			}
			break;
		case 2:
			if (PIN_READ(DEBUG_MODE_ON_PIN)) {
				DebugMode = 1;
				add_channels_on(0);	// Уменьшить количество каналов ADC.
			}
			if (PIN_READ(DEBUG_LCD_ON_PIN)) {
				DebugMode = 0;
				add_channels_on(0);	// Уменьшить количество каналов ADC.
			}

			TCU.EngineWork = 1;
			solenoid_manual_control();		// Ручное управление соленоидами.
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
	Counter = 0;

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
			print_config_gear2_reactivate();
			break;
		case 4:
			print_config_gear3_slu_pressure();
			break;
		case 5:
			print_config_slt_pressure();
			break;	
		case 6:
			print_config_slt_temp_corr();
			break;
		case 7:
			print_config_sln_pressure();
			break;
		case 8:
			print_config_d4_max_gear();
			break;
	}
	ValueDelta = 0;
}

// Основной экран.
static void print_dispay_main() {
	//|12345678901234567890|
	//|O 070| 00 01 |I 1000|
	//|T 120| P 110 |O 0900|
	//|N 120| S D-D |Sp 060|
	//|U 120| Gr  4 |R 1.00|
	//|12345678901234567890|

	char GearRatioChar[5] = {'-', '.', '-', '-', ' '};
	if (TCU.OutputRPM > 100) {
		snprintf(GearRatioChar, 5, "%1u.%02u", 
			MIN(9, TCU.DrumRPM / TCU.OutputRPM), MIN(99, ((TCU.DrumRPM % TCU.OutputRPM) * 100) / TCU.OutputRPM));
	}

	// row,  col
	lcd_set_cursor(0, 0);
	snprintf(LCDArray, 21, "O %3i| %1u%1u %1u%1u |I %4u", 
		CONSTRAIN(TCU.OilTemp, -30, 150), MIN(1, TCU.S1), MIN(1, TCU.S2), MIN(1, TCU.S3), MIN(1, TCU.S4), MIN(9998, TCU.DrumRPM));
	lcd_send_string(LCDArray, 20);

	lcd_set_cursor(1, 0);
	snprintf(LCDArray, 21, "T %3u| A %3u |O %4u",TCU.SLT, TCU.TPS, TCU.OutputRPM);
	lcd_send_string(LCDArray, 20);

	lcd_set_cursor(2, 0);
	snprintf(LCDArray, 21, "N %3u| S %c-%c |Sp %3i", TCU.SLN, ATModeChar[TCU.Selector], ATModeChar[TCU.ATMode], TCU.CarSpeed);
	lcd_send_string(LCDArray, 20);

	lcd_set_cursor(3, 0);
	snprintf(LCDArray, 21, "U %3u| Gr %2i |R %s", TCU.SLU, CONSTRAIN(TCU.Gear, -1, 6), GearRatioChar);
	lcd_send_string(LCDArray, 20);
}

// Экран настройки линейного давления SLT по переключению 3>4.
static void print_config_slt_pressure() {
	//|01234567890123456789|
	//|SLT Press. |100|1.00|
	//|TP-12 TV 12 A99 U101|
	//|  0|  5| 10| 15| 20||
	//| 67| 72| 74| 77| 81||
	//|01234567890123456789|

	// После переключения передачи считываем параметры и обнуляем.
	// Находим по сетке позицию в массиве.
	if (LastGear4ChangeTPS) {
		Gear4ChangeTPS = LastGear4ChangeTPS;
		LastGear4ChangeTPS = 0;
		CursorPos = get_tps_index(Gear4ChangeTPS);

		Gear4ChangeSLT = LastGear4ChangeSLT;
		LastGear4ChangeSLT = 0;
	}

	if (CursorPos >= TPS_GRID_SIZE) {CursorPos = 0;}	// Ограничение по длине массива.
	if (CursorPos < StartCol) {StartCol = CursorPos;}
	if (CursorPos > StartCol + COLUMN_COUNT - 1) {StartCol = CursorPos + 1 - COLUMN_COUNT;}

	char GearRatioChar[5] = {'-', '.', '-', '-', ' '};
	if (TCU.OutputRPM > 100) {
		snprintf(GearRatioChar, 5, "%1u.%02u", 
			MIN(9, TCU.DrumRPM / TCU.OutputRPM), MIN(99, ((TCU.DrumRPM % TCU.OutputRPM) * 100) / TCU.OutputRPM));
	}

	// row,  col
	lcd_set_cursor(0, 0);
	snprintf(LCDArray, 21, "SLT Press. |%3u|%s", TCU.InstTPS, GearRatioChar);
	lcd_send_string(LCDArray, 20);

	// Строка с необходимыми значениями.
	int8_t OilTempCorrSLTP = get_slt_temp_corr(0);				// В %.
	int8_t OilTempCorrSLTV = get_slt_temp_corr(Gear4ChangeSLT);	// В единицах ШИМ.
	lcd_set_cursor(1, 0);
	snprintf(LCDArray, 21, "TP%3i TV%3i A%2u T%3u", 
		CONSTRAIN(OilTempCorrSLTP, -99, 99), CONSTRAIN(OilTempCorrSLTV, -99, 99), MIN(99, Gear4ChangeTPS), Gear4ChangeSLT);
	lcd_send_string(LCDArray, 20);

	// Изменяемые значения.
	for (uint8_t i = 0; i < COLUMN_COUNT; i++) {

		lcd_set_cursor(2, i * 4);
		snprintf(LCDArray, 4, "%3u", TPSGrid[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		lcd_set_cursor(3, i * 4);
		snprintf(LCDArray, 4, "%3u", SLTGraph[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		if (CursorPos == StartCol + i) {
			if (ValueDelta < 0 && SLTGraph[CursorPos] > 5) {SLTGraph[CursorPos] += ValueDelta;}
			if (ValueDelta > 0 && SLTGraph[CursorPos] < 250) {SLTGraph[CursorPos] += ValueDelta;}
			ValueDelta = 0;

			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('<');
		}
		else {
			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('|');	
		}
	}
}

// Экран настройки температурной корекции давления SLT по переключению 3>4.
static void print_config_slt_temp_corr() {
	//|12345678901234567890|
	//|SLT Tmp Cor|100|1.00|
	//|TP-12 TV 12 A99 U101|
	//|  0|  5| 10| 15| 20||
	//| 67| 72| 74| 77| 81||
	//|01234567890123456789|

	// После переключения передачи считываем параметры и обнуляем.
	// Находим по сетке позицию в массиве.
	if (LastGear4ChangeTPS) {
		Gear4ChangeTPS = LastGear4ChangeTPS;
		LastGear4ChangeTPS = 0;
		CursorPos = get_temp_index(TCU.OilTemp);

		Gear4ChangeSLT = LastGear4ChangeSLT;
		LastGear4ChangeSLT = 0;
	}

	if (CursorPos >= TEMP_GRID_SIZE) {CursorPos = 0;}	// Ограничение по длине массива.
	if (CursorPos < StartCol) {StartCol = CursorPos;}
	if (CursorPos > StartCol + COLUMN_COUNT - 1) {StartCol = CursorPos + 1 - COLUMN_COUNT;}

	char GearRatioChar[5] = {'-', '.', '-', '-', ' '};
	if (TCU.OutputRPM > 100) {
		snprintf(GearRatioChar, 5, "%1u.%02u", 
			MIN(9, TCU.DrumRPM / TCU.OutputRPM), MIN(99, ((TCU.DrumRPM % TCU.OutputRPM) * 100) / TCU.OutputRPM));
	}

	// row,  col
	lcd_set_cursor(0, 0);
	snprintf(LCDArray, 21, "SLT Tmp Cor|%3u|%s", TCU.InstTPS, GearRatioChar);
	lcd_send_string(LCDArray, 20);

	// Строка с необходимыми значениями.
	int8_t OilTempCorrSLTP = get_slt_temp_corr(0);				// В %.
	int8_t OilTempCorrSLTV = get_slt_temp_corr(Gear4ChangeSLT);	// В единицах ШИМ.
	lcd_set_cursor(1, 0);
	snprintf(LCDArray, 21, "TP%3i TV%3i A%2u T%3u", 
		CONSTRAIN(OilTempCorrSLTP, -99, 99), CONSTRAIN(OilTempCorrSLTV, -99, 99), MIN(99, Gear4ChangeTPS), Gear4ChangeSLT);
	lcd_send_string(LCDArray, 20);

	// Изменяемые значения.
	for (uint8_t i = 0; i < COLUMN_COUNT; i++) {
		lcd_set_cursor(2, i * 4);
		snprintf(LCDArray, 4, "%3i", TempGrid[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		lcd_set_cursor(3, i * 4);
		snprintf(LCDArray, 4, "%3i", SLTTempCorrGraph[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		if (CursorPos == StartCol + i) {
			if (ValueDelta < 0 && SLTTempCorrGraph[CursorPos] > -60) {SLTTempCorrGraph[CursorPos] += ValueDelta;}
			if (ValueDelta > 0 && SLTTempCorrGraph[CursorPos] < 60) {SLTTempCorrGraph[CursorPos] += ValueDelta;}
			ValueDelta = 0;

			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('<');
		}
		else {
			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('|');	
		}
	}
}

// Экран настройка давления аккумуляторов SLN.
static void print_config_sln_pressure() {
	//|01234567890123456789|
	//|SLN Press. |100|1.00|
	//|TP-12 TV 12 A99 N101|
	//|  0|  5| 10| 15| 20||
	//| 67| 72| 74| 77| 81||
	//|01234567890123456789|

	// После переключения передачи считываем параметры и обнуляем.
	// Находим по сетке позицию в массиве.
	if (LastGear4ChangeTPS) {
		Gear4ChangeTPS = LastGear4ChangeTPS;
		LastGear4ChangeTPS = 0;
		CursorPos = get_tps_index(Gear4ChangeTPS);

		Gear4ChangeSLN = LastGear4ChangeSLN;
		LastGear4ChangeSLN = 0;
	}

	if (CursorPos >= TPS_GRID_SIZE) {CursorPos = 0;}	// Ограничение по длине массива.
	if (CursorPos < StartCol) {StartCol = CursorPos;}
	if (CursorPos > StartCol + COLUMN_COUNT - 1) {StartCol = CursorPos + 1 - COLUMN_COUNT;}

	char GearRatioChar[5] = {'-', '.', '-', '-', ' '};
	if (TCU.OutputRPM > 100) {
		snprintf(GearRatioChar, 5, "%1u.%02u", 
			MIN(9, TCU.DrumRPM / TCU.OutputRPM), MIN(99, ((TCU.DrumRPM % TCU.OutputRPM) * 100) / TCU.OutputRPM));
	}

	// row,  col
	lcd_set_cursor(0, 0);
	snprintf(LCDArray, 21, "SLN Press. |%3u|%s", TCU.InstTPS, GearRatioChar);
	lcd_send_string(LCDArray, 20);

	// Строка с необходимыми значениями.
	int8_t OilTempCorrSLTP = get_slt_temp_corr(0);				// В %.
	int8_t OilTempCorrSLTV = get_slt_temp_corr(Gear4ChangeSLT);	// В единицах ШИМ.
	lcd_set_cursor(1, 0);
	snprintf(LCDArray, 21, "TP%3i TV%3i A%2u T%3u", 
		CONSTRAIN(OilTempCorrSLTP, -99, 99), CONSTRAIN(OilTempCorrSLTV, -99, 99), MIN(99, Gear4ChangeTPS), Gear4ChangeSLN);
	lcd_send_string(LCDArray, 20);

	// Изменяемые значения.
	for (uint8_t i = 0; i < COLUMN_COUNT; i++) {

		lcd_set_cursor(2, i * 4);
		snprintf(LCDArray, 4, "%3u", TPSGrid[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		lcd_set_cursor(3, i * 4);
		snprintf(LCDArray, 4, "%3u", SLNGraph[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		if (CursorPos == StartCol + i) {
			if (ValueDelta < 0 && SLNGraph[CursorPos] > 5) {SLNGraph[CursorPos] += ValueDelta;}
			if (ValueDelta > 0 && SLNGraph[CursorPos] < 250) {SLNGraph[CursorPos] += ValueDelta;}
			ValueDelta = 0;

			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('<');
		}
		else {
			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('|');	
		}
	}
}

// Экран настройки давления в тормозе B3 для включения второй передачи.
static void print_config_gear2_slu_pressure() {
	//|01234567890123456789|
	//|Gear 2 SLU |100|1.00|
	//|UP-12 UV 12 A99 U101|
	//|  0|  5| 10| 15| 20||
	//| 67| 72| 74| 77| 81||
	//|01234567890123456789|

	// После переключения передачи считываем параметры и обнуляем.
	// Находим по ДПДЗ позицию в массиве.
	if (LastGear2ChangeTPS) {
		Gear2ChangeTPS = LastGear2ChangeTPS;
		LastGear2ChangeTPS = 0;
		CursorPos = get_tps_index(Gear2ChangeTPS);

		Gear2ChangeSLU = LastGear2ChangeSLU;
		LastGear2ChangeSLU = 0;
	}

	if (CursorPos >= TPS_GRID_SIZE) {CursorPos = 0;} // Ограничение по длине массива.

	if (CursorPos < StartCol) {StartCol = CursorPos;}
	if (CursorPos > StartCol + COLUMN_COUNT - 1) {StartCol = CursorPos + 1 - COLUMN_COUNT;}

	char GearRatioChar[5] = {'-', '.', '-', '-', ' '};
	if (TCU.OutputRPM > 100) {
		snprintf(GearRatioChar, 5, "%1u.%02u", 
			MIN(9, TCU.DrumRPM / TCU.OutputRPM), MIN(99, ((TCU.DrumRPM % TCU.OutputRPM) * 100) / TCU.OutputRPM));
	}

	// row,  col
	lcd_set_cursor(0, 0);
	snprintf(LCDArray, 21, "Gear 2 SLU |%3u|%s", TCU.InstTPS, GearRatioChar);
	lcd_send_string(LCDArray, 20);

	// Строка с необходимыми значениями.
	int8_t OilTempCorrSLUP = get_slu_gear2_temp_corr(0);				// В %.
	int8_t OilTempCorrSLUV = get_slu_gear2_temp_corr(Gear2ChangeSLU);	// В единицах ШИМ.
	lcd_set_cursor(1, 0);
	snprintf(LCDArray, 21, "UP%3i UV%3i A%2u U%3u", 
		CONSTRAIN(OilTempCorrSLUP, -99, 99), CONSTRAIN(OilTempCorrSLUV, -99, 99), MIN(99, Gear2ChangeTPS), Gear2ChangeSLU);
	lcd_send_string(LCDArray, 20);

	// Изменяемые значения.
	for (uint8_t i = 0; i < COLUMN_COUNT; i++) {
		lcd_set_cursor(2, i * 4);
		snprintf(LCDArray, 4, "%3u", TPSGrid[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		lcd_set_cursor(3, i * 4);
		snprintf(LCDArray, 4, "%3u", SLUGear2Graph[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		if (CursorPos == StartCol + i) {
			if (ValueDelta < 0 && SLUGear2Graph[CursorPos] > 5) {SLUGear2Graph[CursorPos] += ValueDelta;}
			if (ValueDelta > 0 && SLUGear2Graph[CursorPos] < 250) {SLUGear2Graph[CursorPos] += ValueDelta;}
			ValueDelta = 0;

			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('<');
		}
		else {
			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('|');	
		}
	}
}

// Экран настройки температурной корекции давления SLU по переключению 1>2.
static void print_config_gear2_slu_temp_corr() {
	//|12345678901234567890|
	//|SLU G2 TCor|100|1.00|
	//|UP-12 UV 12 A99 U101|
	//|  0|  5| 10| 15| 20||
	//| 67| 72| 74| 77| 81||
	//|01234567890123456789|

	// После переключения передачи считываем параметры и обнуляем.
	// Находим по ДПДЗ позицию в массиве.
	if (LastGear2ChangeTPS) {
		Gear2ChangeTPS = LastGear2ChangeTPS;
		LastGear2ChangeTPS = 0;
		CursorPos = get_temp_index(TCU.OilTemp);

		Gear2ChangeSLU = LastGear2ChangeSLU;
		LastGear2ChangeSLU = 0;
	}

	if (CursorPos >= TEMP_GRID_SIZE) {CursorPos = 0;}	// Ограничение по длине массива.
	if (CursorPos < StartCol) {StartCol = CursorPos;}
	if (CursorPos > StartCol + COLUMN_COUNT - 1) {StartCol = CursorPos + 1 - COLUMN_COUNT;}

	char GearRatioChar[5] = {'-', '.', '-', '-', ' '};
	if (TCU.OutputRPM > 100) {
		snprintf(GearRatioChar, 5, "%1u.%02u", 
			MIN(9, TCU.DrumRPM / TCU.OutputRPM), MIN(99, ((TCU.DrumRPM % TCU.OutputRPM) * 100) / TCU.OutputRPM));
	}

	// row,  col
	lcd_set_cursor(0, 0);
	snprintf(LCDArray, 21, "SLU G2 TCor|%3u|%s", TCU.InstTPS, GearRatioChar);
	lcd_send_string(LCDArray, 20);

	// Строка с необходимыми значениями.
	int8_t OilTempCorrSLUP = get_slu_gear2_temp_corr(0);				// В %.
	int8_t OilTempCorrSLUV = get_slu_gear2_temp_corr(Gear2ChangeSLU);	// В единицах ШИМ.
	lcd_set_cursor(1, 0);
	snprintf(LCDArray, 21, "UP%3i UV%3i A%2u U%3u", 
		CONSTRAIN(OilTempCorrSLUP, -99, 99), CONSTRAIN(OilTempCorrSLUV, -99, 99), MIN(99, Gear2ChangeTPS), Gear2ChangeSLU);
	lcd_send_string(LCDArray, 20);

	// Изменяемые значения.
	for (uint8_t i = 0; i < COLUMN_COUNT; i++) {
		lcd_set_cursor(2, i * 4);
		snprintf(LCDArray, 4, "%3i", TempGrid[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		lcd_set_cursor(3, i * 4);
		snprintf(LCDArray, 4, "%3i", SLUGear2TempCorrGraph[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		if (CursorPos == StartCol + i) {
			if (ValueDelta < 0 && SLUGear2TempCorrGraph[CursorPos] > -60) {SLUGear2TempCorrGraph[CursorPos] += ValueDelta;}
			if (ValueDelta > 0 && SLUGear2TempCorrGraph[CursorPos] < 60) {SLUGear2TempCorrGraph[CursorPos] += ValueDelta;}
			ValueDelta = 0;

			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('<');
		}
		else {
			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('|');	
		}
	}
}

// Экран настройки опережения включения второй передачи после ХХ.
static void print_config_gear2_reactivate() {
	//|12345678901234567890|
	//|G2 REACTIV |100|1.00|
	//|    A 99 | D101     |
	//|  0|  5| 10| 15| 20||
	//| 67| 72| 74| 77| 81||
	//|12345678901234567890|

	// После переключения передачи считываем параметры и обнуляем.
	// Находим по ДПДЗ позицию в массиве.
	if (LastGear2ReactivateTPS) {
		Gear2ReactivateTPS = LastGear2ReactivateTPS;
		LastGear2ReactivateTPS = 0;
		CursorPos = get_tps_index(Gear2ReactivateTPS);

		Gear2ReactivateRPM = LastGear2ReactivateRPM;
		LastGear2ReactivateRPM = 0;
	}

	if (CursorPos >= TPS_GRID_SIZE) {CursorPos = 0;} // Ограничение по длине массива.

	if (CursorPos < StartCol) {StartCol = CursorPos;}
	if (CursorPos > StartCol + COLUMN_COUNT - 1) {StartCol = CursorPos + 1 - COLUMN_COUNT;}

	char GearRatioChar[5] = {'-', '.', '-', '-', ' '};
	if (TCU.OutputRPM > 100) {
		snprintf(GearRatioChar, 5, "%1u.%02u",
			MIN(9, TCU.DrumRPM / TCU.OutputRPM), MIN(99, ((TCU.DrumRPM % TCU.OutputRPM) * 100) / TCU.OutputRPM));
	}

	// row,  col
	lcd_set_cursor(0, 0);
	snprintf(LCDArray, 21, "G2 REACTIV |%3u|%s", TCU.InstTPS, GearRatioChar);
	lcd_send_string(LCDArray, 20);

	// Строка с необходимыми значениями.
	lcd_set_cursor(1, 0);
	snprintf(LCDArray, 21, "    A %2u | D%3u     ", MIN(99, Gear2ReactivateTPS), MIN(999, Gear2ReactivateRPM));
	lcd_send_string(LCDArray, 20);

	// Изменяемые значения.
	for (uint8_t i = 0; i < COLUMN_COUNT; i++) {
		lcd_set_cursor(2, i * 4);
		snprintf(LCDArray, 4, "%3u", TPSGrid[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		lcd_set_cursor(3, i * 4);
		snprintf(LCDArray, 4, "%3i", Gear2DeltaRPM[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		if (CursorPos == StartCol + i) {
			if (ValueDelta < 0 && Gear2DeltaRPM[CursorPos] > 50) {Gear2DeltaRPM[CursorPos] += ValueDelta * 10;}
			if (ValueDelta > 0 && Gear2DeltaRPM[CursorPos] < 990) {Gear2DeltaRPM[CursorPos] += ValueDelta * 10;}
			ValueDelta = 0;

			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('<');
		}
		else {
			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('|');
		}
	}
}

// Экран настройки задержки выключения SLU при включении третьей передачи.
static void print_config_gear3_slu_pressure() {
	//|12345678901234567890|
	//|G3 SLU Add |100|1.00|
	//|UP-12 UV-12 A99 D101|
	//|  0|  5| 10| 15| 20||
	//| 67| 72| 74| 77| 81||
	//|12345678901234567890|

	// После переключения передачи считываем параметры и обнуляем.
	// Находим по ДПДЗ позицию в массиве.
	if (LastGear3ChangeTPS) {
		Gear3ChangeTPS = LastGear3ChangeTPS;
		LastGear3ChangeTPS = 0;
		CursorPos = get_tps_index(Gear3ChangeTPS);

		Gear3ChangeSLU = LastGear3ChangeSLU;
		LastGear3ChangeSLU = 0;
	}

	if (CursorPos >= TPS_GRID_SIZE) {CursorPos = 0;} // Ограничение по длине массива.

	if (CursorPos < StartCol) {StartCol = CursorPos;}
	if (CursorPos > StartCol + COLUMN_COUNT - 1) {StartCol = CursorPos + 1 - COLUMN_COUNT;}

	char GearRatioChar[5] = {'-', '.', '-', '-', ' '};
	if (TCU.OutputRPM > 100) {
		snprintf(GearRatioChar, 5, "%1u.%02u", 
			MIN(9, TCU.DrumRPM / TCU.OutputRPM), MIN(99, ((TCU.DrumRPM % TCU.OutputRPM) * 100) / TCU.OutputRPM));
	}

	// row,  col
	lcd_set_cursor(0, 0);
	snprintf(LCDArray, 21, "G3 SLU Add |%3u|%s", TCU.InstTPS, GearRatioChar);
	lcd_send_string(LCDArray, 20);

	// Строка с необходимыми значениями.
	int8_t OilTempCorrSLUP = get_slu_gear2_temp_corr(0);				// В %.
	int8_t OilTempCorrSLUV = get_slu_gear2_temp_corr(Gear2ChangeSLU);	// В единицах ШИМ.
	lcd_set_cursor(1, 0);
	snprintf(LCDArray, 21, "UC%3i UV%3i A%2u D%3u", 
		CONSTRAIN(OilTempCorrSLUP, -99, 99), CONSTRAIN(OilTempCorrSLUV, -99, 99), MIN(99, Gear3ChangeTPS), Gear3ChangeSLU);
	lcd_send_string(LCDArray, 20);

	// Изменяемые значения.
	for (uint8_t i = 0; i < COLUMN_COUNT; i++) {
		lcd_set_cursor(2, i * 4);
		snprintf(LCDArray, 4, "%3u", TPSGrid[StartCol + i]);
		lcd_send_string(LCDArray, 3);

		lcd_set_cursor(3, i * 4);
		int8_t Value = SLUGear3AddGraph[StartCol + i];
		snprintf(LCDArray, 4, "%3i", Value);
		lcd_send_string(LCDArray, 3);

		if (CursorPos == StartCol + i) {
			if (ValueDelta < 0) {SLUGear3AddGraph[CursorPos] += ValueDelta;}
			if (ValueDelta > 0) {SLUGear3AddGraph[CursorPos] += ValueDelta;}
			SLUGear3AddGraph[CursorPos] = CONSTRAIN(SLUGear3AddGraph[CursorPos], -32, 64);
			ValueDelta = 0;

			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('<');
		}
		else {
			lcd_set_cursor(2, i * 4 + 3);
			lcd_send_char('|');
			lcd_set_cursor(3, i * 4 + 3);
			lcd_send_char('|');	
		}
	}
}

// Временная установка максимальной передачи в режиме D4.
static void print_config_d4_max_gear() {
	//|12345678901234567890|
	//|    D4 Max Gear     |
	//|O 99 A 99 U 101 C -3|
	//|         4          |
	//|                    |
	//|12345678901234567890|

	// row,  col
	lcd_set_cursor(0, 0);
	lcd_send_string("    D4 Max Gear     ", 20);	// Заголовок.
	lcd_set_cursor(1, 0);
	lcd_send_string("                    ", 20);	// Заголовок.

	lcd_set_cursor(2, 0);
	snprintf(LCDArray, 21, "         %1u          ", MaxGear[5]);
	lcd_send_string(LCDArray, 20);

	lcd_set_cursor(3, 0);
	lcd_send_string("                    ", 20);

	if (ValueDelta) {
		MaxGear[5] = CONSTRAIN(MaxGear[5] + ValueDelta, 1, 4);
		ValueDelta = 0;
	}
}

static uint8_t get_tps_index(uint8_t TPS) {
	if (TPS < 3) {return 0;}

	uint8_t Delta = 255;
	for (uint8_t i = 1; i < TPS_GRID_SIZE; i++) {
		uint8_t Diff = 0;
		if (TPS > TPSGrid[i]) {Diff = TPS - TPSGrid[i];}
		else {Diff = TPSGrid[i] - TPS;}

		if (Diff < Delta) {Delta = Diff;}
		else {return i - 1;}
	}
	return 0;
}

static uint8_t get_temp_index(int16_t Temp) {
	if (Temp < -27) {return 0;}

	uint8_t Delta = 255;
	for (uint8_t i = 1; i < TEMP_GRID_SIZE; i++) {
		uint8_t Diff = 0;
		if (Temp > TempGrid[i]) {Diff = Temp - TempGrid[i];}
		else {Diff = TempGrid[i] - Temp;}

		if (Diff < Delta) {Delta = Diff;}
		else {return i - 1;}
	}
	return 0;
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
	TCU.SLT = get_adc_value(2) >> 2;
	if (TCU.SLT <= 2) {TCU.SLT = 0;}
	if (TCU.SLT >= 253) {TCU.SLT = 255;}

	TCU.SLN = get_adc_value(3) >> 2;
	if (TCU.SLN <= 2) {TCU.SLN = 0;}
	if (TCU.SLN >= 253) {TCU.SLN = 255;}

	TCU.SLU = 50 + (get_adc_value(4) >> 3);

	// Устанавливаем ШИМ на соленоидах.
	OCR1A = TCU.SLT;
	OCR1B = TCU.SLN;
	OCR1C = TCU.SLU;
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

	if (ButtonState[4] == 201) {	// SLN Смена экрана.
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

	if (PIN_READ(DEBUG_SCREEN_BUTTON_PIN) && ButtonState[4] > 200) {ButtonState[4] = 100;}
}

// Вызов каждые 50 мс.
static void buttons_update() {
	button_read(0, PIN_READ(DEBUG_S1_PIN));
	button_read(1, PIN_READ(DEBUG_S2_PIN));
	button_read(2, PIN_READ(DEBUG_S3_PIN));
	button_read(3, PIN_READ(DEBUG_S4_PIN));

	button_read(4, PIN_READ(DEBUG_SCREEN_BUTTON_PIN));
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
