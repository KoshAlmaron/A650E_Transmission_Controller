#include <avr/eeprom.h>		// EEPROM.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "uart.h"			// UART.

#include "eeprom.h"			// Свой заголовок.

// TPS_GRID_SIZE	21 	 	42
// TEMP_GRID_SIZE	31 		62
// Чтение EEPROM

void read_eeprom() {
	//update_eeprom();

	// В ячейке 2048 хранится байт инициализации, если он равен 0xab,
	// то при старте значения из прошивки записываются в EEPROM.
	uint8_t DataInit = eeprom_read_byte((uint8_t*) 2048);
	if (DataInit == TABLES_INIT_COMMAND) {
		update_eeprom();
		eeprom_update_byte((uint8_t*) 2048, 0x00);
		uart_send_table(0);
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

	// 772-813 - TPSGraph.
	eeprom_read_block((void*)&TPSGraph, (const void*) 772, TPS_GRID_SIZE * 2);
	// 814-875 - OilTempGraph.
	eeprom_read_block((void*)&OilTempGraph, (const void*) 814, TEMP_GRID_SIZE * 2);
}

// Запись EEPROM.
void update_eeprom() {
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

	// 772-813 - TPSGraph.
	eeprom_update_block((void*)&TPSGraph, (void*) 772, TPS_GRID_SIZE * 2);
	// 814-875 - OilTempGraph.
	eeprom_update_block((void*)&OilTempGraph, (void*) 814, TEMP_GRID_SIZE * 2);
}
