#include <avr/eeprom.h>
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "eeprom.h"			// Свой заголовок.

// Чтение EEPROM
void read_eeprom() {
	//update_eeprom();

	// Адрес массива, адрес ячейки, кол-во байт.
	// 0-41 - SLUGear2Graph.
	eeprom_read_block((void*)&SLUGear2Graph, (const void*) 0, TPS_GRID_SIZE * 2);
	// 42-115 - Gear2TempCorrGraph.
	eeprom_read_block((void*)&Gear2TempCorrGraph, (const void*) 42, TEMP_GRID_SIZE * 2);
	// 116-157 - SLTGear3Graph.
	eeprom_read_block((void*)&SLTGear3Graph, (const void*) 116, TPS_GRID_SIZE * 2);
}

// Запись EEPROM.
void update_eeprom() {
	// 0-41 - SLUGear2Graph.
	eeprom_update_block((void*)&SLUGear2Graph, (void*) 0, TPS_GRID_SIZE * 2);
	// 42-115 - Gear2TempCorrGraph.
	eeprom_update_block((void*)&Gear2TempCorrGraph, (void*) 42, TEMP_GRID_SIZE * 2);
	// 116-157 - SLTGear3Graph.
	eeprom_update_block((void*)&SLTGear3Graph, (void*) 116, TPS_GRID_SIZE * 2);
}