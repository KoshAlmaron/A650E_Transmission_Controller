#include <avr/eeprom.h>		// EEPROM.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "uart.h"			// UART.

#include "configuration.h"	// Настройки.
#include "eeprom.h"			// Свой заголовок.

/*
	TPS_GRID_SIZE	21 	 	42
	TEMP_GRID_SIZE	31 		62

0-2047		- Таблицы
	0-1600		- Основные
	1601-1810 	- АЦП
	1811-2047	- Скорость
2048-3071	- Настройки
3072-4095	- Флаги и прочий хлам

*/

// ========================== Таблицы с адаптацией ============================
void update_eeprom_adaptation() {
	// 292-333 SLUGear2TPSAdaptGraph.
	eeprom_update_block((void*)&SLUGear2TPSAdaptGraph, (void*) 292, TPS_GRID_SIZE * 2);
	// 334-395 SLUGear2TempAdaptGraph.
	eeprom_update_block((void*)&SLUGear2TempAdaptGraph, (void*) 334, TEMP_GRID_SIZE * 2);

	// 626-667 - SLUGear3TPSAdaptGraph.
	eeprom_update_block((void*)&SLUGear3TPSAdaptGraph, (void*) 626, TPS_GRID_SIZE * 2);
	// 668-729 - SLUGear3TempAdaptGraph.
	eeprom_update_block((void*)&SLUGear3TempAdaptGraph, (void*) 668, TEMP_GRID_SIZE * 2);

	// 730-771 - Gear2AdvAdaptGraph.
	eeprom_update_block((void*)&Gear2AdvAdaptGraph, (void*) 730, DELTA_RPM_GRID_SIZE * 2);
	// 834-895 - Gear2AdvTempAdaptGraph.
	eeprom_update_block((void*)&Gear2AdvTempAdaptGraph, (void*) 834, TEMP_GRID_SIZE * 2);
}

// ============================ Основные таблицы ==============================
// Чтение EEPROM
void read_eeprom_tables() {
	// В ячейке 3072 хранится байт инициализации, если он равен 0xab (OVERWRITE_BYTE),
	// то при старте значения из прошивки записываются в EEPROM.
	uint8_t DataInit = eeprom_read_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 0);
	if (DataInit == OVERWRITE_BYTE) {
		update_eeprom_tables();
		eeprom_update_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 0, 0x00);
		uart_send_table(SLT_GRAPH);
		return;
	}

	// Адрес массива, адрес ячейки, кол-во байт.
	// 0-41 - SLTGraph.
	eeprom_read_block((void*)&SLTGraph, (const void*) 0, TPS_GRID_SIZE * 2);
	// 42-103 - SLTTempCorrGraph.
	eeprom_read_block((void*)&SLTTempCorrGraph, (const void*) 42, TEMP_GRID_SIZE * 2);

	// 104-145 - SLNGraph.
	eeprom_read_block((void*)&SLNGraph, (const void*) 104, TPS_GRID_SIZE * 2);
	
	// 146-187 - SLUGear2Graph.
	eeprom_read_block((void*)&SLUGear2Graph, (const void*) 146, TPS_GRID_SIZE * 2);
	// 188-249 - SLUGear2TempCorrGraph.
	eeprom_read_block((void*)&SLUGear2TempCorrGraph, (const void*) 188, TEMP_GRID_SIZE * 2);
	// 250-291 Gear2AdvGraph.
	eeprom_read_block((void*)&Gear2AdvGraph, (const void*) 250, DELTA_RPM_GRID_SIZE * 2);
	// 292-333 SLUGear2TPSAdaptGraph.
	eeprom_read_block((void*)&SLUGear2TPSAdaptGraph, (const void*) 292, TPS_GRID_SIZE * 2);
	// 334-395 SLUGear2TempAdaptGraph.
	eeprom_read_block((void*)&SLUGear2TempAdaptGraph, (const void*) 334, TEMP_GRID_SIZE * 2);

	// 396-437 - SLUGear3Graph.
	eeprom_read_block((void*)&SLUGear3Graph, (const void*) 396, TPS_GRID_SIZE * 2);
	// 438-479 - SLUGear3DelayGraph.
	eeprom_read_block((void*)&SLUGear3DelayGraph, (const void*) 438, TPS_GRID_SIZE * 2);
	// 480-521 - SLNGear3OffsetGraph.
	eeprom_read_block((void*)&SLNGear3OffsetGraph, (const void*) 480, TPS_GRID_SIZE * 2);

	// 522-563 - SLNGear3Graph.
	eeprom_read_block((void*)&SLNGear3Graph, (const void*) 522, TPS_GRID_SIZE * 2);

	// 564-625 - SLUG3DelayTempCorrGraph.
	eeprom_read_block((void*)&SLUG3DelayTempCorrGraph, (const void*) 564, TEMP_GRID_SIZE * 2);
	// 626-667 - SLUGear3TPSAdaptGraph.
	eeprom_read_block((void*)&SLUGear3TPSAdaptGraph, (const void*) 626, TPS_GRID_SIZE * 2);
	// 668-729 - SLUGear3TempAdaptGraph.
	eeprom_read_block((void*)&SLUGear3TempAdaptGraph, (const void*) 668, TEMP_GRID_SIZE * 2);
	// 730-771 - Gear2AdvAdaptGraph.
	eeprom_read_block((void*)&Gear2AdvAdaptGraph, (const void*) 730, DELTA_RPM_GRID_SIZE * 2);

	// 772-833 - Gear2AdvTempCorrGraph.
	eeprom_read_block((void*)&Gear2AdvTempCorrGraph, (const void*) 772, TEMP_GRID_SIZE * 2);
	// 834-895 - Gear2AdvTempAdaptGraph.
	eeprom_read_block((void*)&Gear2AdvTempAdaptGraph, (const void*) 834, TEMP_GRID_SIZE * 2);
}

// Запись EEPROM.
void update_eeprom_tables() {
	// 0-41 - SLTGraph.
	eeprom_update_block((void*)&SLTGraph, (void*) 0, TPS_GRID_SIZE * 2);
	// 42-103 - SLTTempCorrGraph.
	eeprom_update_block((void*)&SLTTempCorrGraph, (void*) 42, TEMP_GRID_SIZE * 2);

	// 104-145 - SLNGraph.
	eeprom_update_block((void*)&SLNGraph, (void*) 104, TPS_GRID_SIZE * 2);

	// 146-187 - SLUGear2Graph.
	eeprom_update_block((void*)&SLUGear2Graph, (void*) 146, TPS_GRID_SIZE * 2);
	// 188-249 - SLUGear2TempCorrGraph.
	eeprom_update_block((void*)&SLUGear2TempCorrGraph, (void*) 188, TEMP_GRID_SIZE * 2);
	// 250-291 Gear2AdvGraph.
	eeprom_update_block((void*)&Gear2AdvGraph, (void*) 250, DELTA_RPM_GRID_SIZE * 2);
	// 292-333 SLUGear2TPSAdaptGraph.
	eeprom_update_block((void*)&SLUGear2TPSAdaptGraph, (void*) 292, TPS_GRID_SIZE * 2);
	// 334-395 SLUGear2TempAdaptGraph.
	eeprom_update_block((void*)&SLUGear2TempAdaptGraph, (void*) 334, TEMP_GRID_SIZE * 2);

	// 396-437 - SLUGear3Graph.
	eeprom_update_block((void*)&SLUGear3Graph, (void*) 396, TEMP_GRID_SIZE * 2);
	// 438-479 - SLUGear3DelayGraph.
	eeprom_update_block((void*)&SLUGear3DelayGraph, (void*) 438, TPS_GRID_SIZE * 2);
	// 480-521 - SLNGear3OffsetGraph.
	eeprom_update_block((void*)&SLNGear3OffsetGraph, (void*) 480, TPS_GRID_SIZE * 2);

	// 522-563 - SLNGear3Graph.
	eeprom_update_block((void*)&SLNGear3Graph, (void*) 522, TPS_GRID_SIZE * 2);

	// 564-625 - SLUG3DelayTempCorrGraph.
	eeprom_update_block((void*)&SLUG3DelayTempCorrGraph, (void*) 564, TEMP_GRID_SIZE * 2);
	// 626-667 - SLUGear3TPSAdaptGraph.
	eeprom_update_block((void*)&SLUGear3TPSAdaptGraph, (void*) 626, TPS_GRID_SIZE * 2);
	// 668-729 - SLUGear3TempAdaptGraph.
	eeprom_update_block((void*)&SLUGear3TempAdaptGraph, (void*) 668, TEMP_GRID_SIZE * 2);
	// 730-771 - Gear2AdvAdaptGraph.
	eeprom_update_block((void*)&Gear2AdvAdaptGraph, (void*) 730, DELTA_RPM_GRID_SIZE * 2);

	// 772-833 - Gear2AdvTempCorrGraph.
	eeprom_update_block((void*)&Gear2AdvTempCorrGraph, (void*) 772, TEMP_GRID_SIZE * 2);
	// 834-895 - Gear2AdvTempAdaptGraph.
	eeprom_update_block((void*)&Gear2AdvTempAdaptGraph, (void*) 834, TEMP_GRID_SIZE * 2);
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

	// 1601-1642 - TPSGraph.
	eeprom_read_block((void*)&TPSGraph, (const void*) 1601, TPS_GRID_SIZE * 2);
	// 1643-1704 - OilTempGraph.
	eeprom_read_block((void*)&OilTempGraph, (const void*) 1643, TEMP_GRID_SIZE * 2);
}

// Запись EEPROM.
void update_eeprom_adc() {
	// 1601-1642 - TPSGraph.
	eeprom_update_block((void*)&TPSGraph, (void*) 1601, TPS_GRID_SIZE * 2);
	// 1643-1704 - OilTempGraph.
	eeprom_update_block((void*)&OilTempGraph, (void*) 1643, TEMP_GRID_SIZE * 2);
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

	// 1811-1978 - Gear_2_1 - Gear_4_5.
	eeprom_read_block((void*)&Gear_2_1, (const void*) 1811 + 21 * 0, TPS_GRID_SIZE);
	eeprom_read_block((void*)&Gear_1_2, (const void*) 1811 + 21 * 1, TPS_GRID_SIZE);
	eeprom_read_block((void*)&Gear_3_2, (const void*) 1811 + 21 * 2, TPS_GRID_SIZE);
	eeprom_read_block((void*)&Gear_2_3, (const void*) 1811 + 21 * 3, TPS_GRID_SIZE);
	eeprom_read_block((void*)&Gear_4_3, (const void*) 1811 + 21 * 4, TPS_GRID_SIZE);
	eeprom_read_block((void*)&Gear_3_4, (const void*) 1811 + 21 * 5, TPS_GRID_SIZE);
	eeprom_read_block((void*)&Gear_5_4, (const void*) 1811 + 21 * 6, TPS_GRID_SIZE);
	eeprom_read_block((void*)&Gear_4_5, (const void*) 1811 + 21 * 7, TPS_GRID_SIZE);
}

void update_eeprom_speed() {
	// 1811-1978 - Gear_2_1 - Gear_4_5.
	eeprom_update_block((void*)&Gear_2_1, (void*) 1811 + 21 * 0, TPS_GRID_SIZE);
	eeprom_update_block((void*)&Gear_1_2, (void*) 1811 + 21 * 1, TPS_GRID_SIZE);
	eeprom_update_block((void*)&Gear_3_2, (void*) 1811 + 21 * 2, TPS_GRID_SIZE);
	eeprom_update_block((void*)&Gear_2_3, (void*) 1811 + 21 * 3, TPS_GRID_SIZE);
	eeprom_update_block((void*)&Gear_4_3, (void*) 1811 + 21 * 4, TPS_GRID_SIZE);
	eeprom_update_block((void*)&Gear_3_4, (void*) 1811 + 21 * 5, TPS_GRID_SIZE);
	eeprom_update_block((void*)&Gear_5_4, (void*) 1811 + 21 * 6, TPS_GRID_SIZE);
	eeprom_update_block((void*)&Gear_4_5, (void*) 1811 + 21 * 7, TPS_GRID_SIZE);
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

	// Структура с настройками 2048-????.
	eeprom_read_block((void*)&CFG, (const void*) 2048, sizeof(CFG));
}

void update_eeprom_config() {
	// Структура с настройками 2048-????.
	eeprom_update_block((void*)&CFG, (void*) 2048, sizeof(CFG));
}


