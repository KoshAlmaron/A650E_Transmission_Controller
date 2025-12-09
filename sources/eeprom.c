#include <avr/eeprom.h>		// EEPROM.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "uart.h"			// UART.

#include "configuration.h"	// Настройки.
#include "eeprom.h"			// Свой заголовок.

/*
	TPS_GRID_SIZE	21 	 	42
	TEMP_GRID_SIZE	31 		62

0-2047		- Таблицы
	0-1400		- Основные
	1401-1600 	- АЦП
	1601-1800	- Скорость
	1801-2047	- Адаптация
2048-3071	- Настройки
3072-4095	- Флаги и прочий хлам
*/

#define TABLES_START_BYTE_MAIN	0
#define TABLES_START_BYTE_ADC	1401
#define TABLES_START_BYTE_SPEED 1601
#define TABLES_START_BYTE_ADAPT 1801

#define CONFIG_START_BYTE		2048
#define OTHER_START_BYTE		3072

// ========================== Таблицы с адаптацией ============================
void read_eeprom_adaptation() {
	eeprom_read_block((void*)&ADAPT, (const void*) TABLES_START_BYTE_ADAPT, sizeof(ADAPT));
}

void update_eeprom_adaptation() {
	eeprom_update_block((void*)&ADAPT, (void*) TABLES_START_BYTE_ADAPT, sizeof(ADAPT));
}

// ============================ Основные таблицы ==============================
void read_eeprom_tables() {
	// В ячейке 3072 хранится байт инициализации, если он равен 0xab (OVERWRITE_BYTE),
	// то при старте значения из прошивки записываются в EEPROM.
	uint8_t DataInit = eeprom_read_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 0);
	if (DataInit == OVERWRITE_BYTE) {
		update_eeprom_tables();		// Сброс таблиц.
		update_eeprom_adaptation();	// Сброс адаптации.
		eeprom_update_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 0, 0x00);
		uart_send_table(SLT_GRAPH);
		return;
	}
	eeprom_read_block((void*)&TABLES, (const void*) TABLES_START_BYTE_MAIN, sizeof(TABLES));
}

// Запись EEPROM.
void update_eeprom_tables() {
	eeprom_update_block((void*)&TABLES, (void*) TABLES_START_BYTE_MAIN, sizeof(TABLES));
}

// ============================== Таблицы АЦП =================================
void read_eeprom_adc() {
	uint8_t DataInit = eeprom_read_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 1);
	if (DataInit == OVERWRITE_BYTE) {
		update_eeprom_adc();
		eeprom_update_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 1, 0x00);
		uart_send_table(TPS_ADC_GRAPH);
		return;
	}
	eeprom_read_block((void*)&ADCTBL, (const void*) TABLES_START_BYTE_ADC, sizeof(ADCTBL));
}

void update_eeprom_adc() {
	eeprom_update_block((void*)&ADCTBL, (void*) TABLES_START_BYTE_ADC, sizeof(ADCTBL));
}


// ================ Таблицы скоростей переключения передач ====================
void read_eeprom_speed() {
	uint8_t DataInit = eeprom_read_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 2);
	if (DataInit == OVERWRITE_BYTE) {
		update_eeprom_speed();
		eeprom_update_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 2, 0x00);
		uart_send_table(GEAR_SPEED_GRAPHS);
		return;
	}
	eeprom_read_block((void*)&SPEED, (const void*) TABLES_START_BYTE_SPEED, sizeof(SPEED));
}

void update_eeprom_speed() {
	eeprom_update_block((void*)&SPEED, (void*) TABLES_START_BYTE_SPEED, sizeof(SPEED));
}

// =========================== Таблицы настроек ===============================
void read_eeprom_config() {
	uint8_t DataInit = eeprom_read_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 3);
	if (DataInit == OVERWRITE_BYTE) {
		update_eeprom_speed();
		eeprom_update_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 3, 0x00);
		uart_send_cfg_data();
		return;
	}

	eeprom_read_block((void*)&CFG, (const void*) CONFIG_START_BYTE, sizeof(CFG));
}

void update_eeprom_config() {
	eeprom_update_block((void*)&CFG, (void*) CONFIG_START_BYTE, sizeof(CFG));
}


