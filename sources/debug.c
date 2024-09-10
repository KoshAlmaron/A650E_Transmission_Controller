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

char LCDArray[21] = {0};	// Массив для отправки на дисплей.

// Обозначение режимов на экране.
int8_t ATModeChar[] = {'I', 'P', 'R', 'N', 'D', '4', '3', '2', 'L', 'E', 'M'};

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

void debug_lcd_init() {
	lcd_init(0x3f);
}

void debug_print_data() {
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
	snprintf(LCDArray, 21, "T %3u| A %3u |O %4u", TCU.SLT, TCU.TPS, TCU.OutputRPM);
	lcd_send_string(LCDArray, 20);

	lcd_set_cursor(2, 0);
	snprintf(LCDArray, 21, "N %3u| S %c-%c |Sp %3i", TCU.SLN, ATModeChar[TCU.Selector], ATModeChar[TCU.ATMode], TCU.CarSpeed);
	lcd_send_string(LCDArray, 20);

	lcd_set_cursor(3, 0);
	snprintf(LCDArray, 21, "U %3u| Gr %2i |R %s", TCU.SLU, CONSTRAIN(TCU.Gear, -1, 6), GearRatioChar);
	lcd_send_string(LCDArray, 20);
}

void solenoid_manual_control() {
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

	TCU.SLU = 50 + (get_adc_value(4) >> 5);

	// Устанавливаем ШИМ на соленоидах.
	OCR1A = TCU.SLT;
	OCR1B = TCU.SLN;
	OCR1C = TCU.SLU;
}