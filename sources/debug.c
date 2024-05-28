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

	char GearRatioChar[5] = {0};
	float GearRatio = 0.00;
	if (TCU.OutputRPM > 1) {GearRatio = (float) TCU.DrumRPM / TCU.OutputRPM;}
	dtostrf(GearRatio, 4, 2, GearRatioChar);

	uint8_t S1 = PIN_READ(SOLENOID_S1_PIN) ? 1 : 0;
	uint8_t S2 = PIN_READ(SOLENOID_S2_PIN) ? 1 : 0;
	uint8_t S3 = PIN_READ(SOLENOID_S3_PIN) ? 1 : 0;
	uint8_t S4 = PIN_READ(SOLENOID_S4_PIN) ? 1 : 0;

	// row,  col
	lcd_set_cursor(0, 0);
	snprintf(LCDArray, 21, "O %3i| %1i%1i %1i%1i |I %4i", TCU.OilTemp, S1, S2, S3, S4, TCU.DrumRPM);
	lcd_send_string(LCDArray, 20);

	lcd_set_cursor(1, 0);
	snprintf(LCDArray, 21, "T %3i| A %3i |O %4i", TCU.SLT, TCU.TPS, TCU.OutputRPM);
	lcd_send_string(LCDArray, 20);

	lcd_set_cursor(2, 0);
	snprintf(LCDArray, 21, "N %3i| S %c-%c |Sp %3i", TCU.SLN, ATModeChar[TCU.Selector], ATModeChar[TCU.ATMode], get_car_speed());
	lcd_send_string(LCDArray, 20);

	lcd_set_cursor(3, 0);
	snprintf(LCDArray, 21, "U %3i| Gr  %1i |R %s", TCU.SLU, TCU.Gear, GearRatioChar);
	lcd_send_string(LCDArray, 20);
}

void solenoid_manual_control() {
	// Считываем положение потенциометров.
	TCU.SLT = get_adc_value(3);
	TCU.SLN = get_adc_value(4);
	TCU.SLU = get_adc_value(5);

	// Устанавливаем ШИМ на соленоидах.
	OCR1A = TCU.SLT;
	OCR1B = TCU.SLN;
	OCR1C = TCU.SLU;
}