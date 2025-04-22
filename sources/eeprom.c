#include <avr/eeprom.h>
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "uart.h"			// UART.

#include "eeprom.h"			// Свой заголовок.

// TPS_GRID_SIZE	21 	 	42
// TEMP_GRID_SIZE	37 		74
// Чтение EEPROM

void read_eeprom() {
	//update_eeprom();
	// Адрес массива, адрес ячейки, кол-во байт.

	//eeprom_update_block((void*)&SLNGraph, (void*) 274, TPS_GRID_SIZE * 2);
	
	// 0-41 - SLTGraph.
	eeprom_read_block((void*)&SLTGraph, (const void*) 0, TPS_GRID_SIZE * 2);
	// 42-115 - SLTTempCorrGraph.
	eeprom_read_block((void*)&SLTTempCorrGraph, (const void*) 42, TEMP_GRID_SIZE * 2);
	// 116-157 - SLUGear2Graph.
	eeprom_read_block((void*)&SLUGear2Graph, (const void*) 116, TPS_GRID_SIZE * 2);
	// 158-199 - SLUGear3DelayGraph.
	eeprom_read_block((void*)&SLUGear3DelayGraph, (const void*) 158, TPS_GRID_SIZE * 2);
	// 200-273 - SLUGear2TempCorrGraph.
	eeprom_read_block((void*)&SLUGear2TempCorrGraph, (const void*) 200, TEMP_GRID_SIZE * 2);
	// 274-315 - SLNGraph.
	eeprom_read_block((void*)&SLNGraph, (const void*) 274, TPS_GRID_SIZE * 2);
	// 316-357 - Gear2DeltaRPM.
	eeprom_read_block((void*)&Gear2DeltaRPM, (const void*) 316, TPS_GRID_SIZE * 2);

	// 358-399 SLUGear2AdoptGraph.
	eeprom_read_block((void*)&SLUGear2TPSAdaptGraph, (const void*) 358, TPS_GRID_SIZE * 2);
	// 400-473 SLUGear2TempAdoptGraph.
	eeprom_read_block((void*)&SLUGear2TempAdaptGraph, (const void*) 400, TEMP_GRID_SIZE * 2);
}

// Запись EEPROM.
void update_eeprom() {
	// 0-41 - SLTGraph.
	eeprom_update_block((void*)&SLTGraph, (void*) 0, TPS_GRID_SIZE * 2);
	// 42-115 - SLTTempCorrGraph.
	eeprom_update_block((void*)&SLTTempCorrGraph, (void*) 42, TEMP_GRID_SIZE * 2);
	// 116-157 - SLUGear2Graph.
	eeprom_update_block((void*)&SLUGear2Graph, (void*) 116, TPS_GRID_SIZE * 2);
	// 158-199 - SLUGear3DelayGraph.
	eeprom_update_block((void*)&SLUGear3DelayGraph, (void*) 158, TPS_GRID_SIZE * 2);
	// 200-273 - SLUGear2TempCorrGraph.
	eeprom_update_block((void*)&SLUGear2TempCorrGraph, (void*) 200, TEMP_GRID_SIZE * 2);
	// 274-315 - SLNGraph.
	eeprom_update_block((void*)&SLNGraph, (void*) 274, TPS_GRID_SIZE * 2);
	// 316-357 - Gear2DeltaRPM.
	eeprom_update_block((void*)&Gear2DeltaRPM, (void*) 316, TPS_GRID_SIZE * 2);

	// 358-399 SLUGear2AdoptGraph.
	eeprom_update_block((void*)&SLUGear2TPSAdaptGraph, (void*) 358, TPS_GRID_SIZE * 2);
	// 400-473 SLUGear2TempAdoptGraph.
	eeprom_update_block((void*)&SLUGear2TempAdaptGraph, (void*) 400, TEMP_GRID_SIZE * 2);

	// Пихаем все в UART, чтобы потом можно было копипастить.
	send_eeprom_to_uart();
}
