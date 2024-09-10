#include <avr/eeprom.h>
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "eeprom.h"			// Свой заголовок.



// TPS_GRID_SIZE	21 	 	42
// TEMP_GRID_SIZE	37 		74
// Чтение EEPROM
void read_eeprom() {
	//update_eeprom();
	// Адрес массива, адрес ячейки, кол-во байт.

	// 0-41 - SLTGraph.
	eeprom_read_block((void*)&SLTGraph, (const void*) 0, TPS_GRID_SIZE * 2);
	// 42-115 - SLTTempCorrGraph.
	eeprom_read_block((void*)&SLTTempCorrGraph, (const void*) 42, TEMP_GRID_SIZE * 2);
	// 116-157 - SLNGraph.
	eeprom_read_block((void*)&SLNGraph, (const void*) 116, TPS_GRID_SIZE * 2);
	// 158-199 - SLUGear2Graph.
	eeprom_read_block((void*)&SLUGear2Graph, (const void*) 158, TPS_GRID_SIZE * 2);
	// 200-241 - SLUGear3Graph.
	eeprom_read_block((void*)&SLUGear3Graph, (const void*) 200, TPS_GRID_SIZE * 2);
	// 242-315 - SLUGear2TempCorrGraph.
	eeprom_read_block((void*)&SLUGear2TempCorrGraph, (const void*) 242, TEMP_GRID_SIZE * 2);
}

// Запись EEPROM.
void update_eeprom() {
	// 0-41 - SLTGraph.
	eeprom_update_block((void*)&SLTGraph, (void*) 0, TPS_GRID_SIZE * 2);
	// 42-115 - SLTTempCorrGraph.
	eeprom_update_block((void*)&SLTTempCorrGraph, (void*) 42, TEMP_GRID_SIZE * 2);
	// 116-157 - SLNGraph.
	eeprom_update_block((void*)&SLNGraph, (void*) 116, TPS_GRID_SIZE * 2);
	// 158-199 - SLUGear2Graph.
	eeprom_update_block((void*)&SLUGear2Graph, (void*) 158, TPS_GRID_SIZE * 2);
	// 200-241 - SLUGear3Graph.
	eeprom_update_block((void*)&SLUGear3Graph, (void*) 200, TPS_GRID_SIZE * 2);
	// 242-315 - SLUGear2TempCorrGraph.
	eeprom_update_block((void*)&SLUGear2TempCorrGraph, (void*) 242, TEMP_GRID_SIZE * 2);
}