#include <avr/eeprom.h>
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "uart.h"			// UART.

#include "eeprom.h"			// Свой заголовок.

// TPS_GRID_SIZE	21 	 	42
// TEMP_GRID_SIZE	31 		62
// Чтение EEPROM

void read_eeprom() {
	//update_eeprom();
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
	// 250-291 SLUGear2AddGraph.
	eeprom_read_block((void*)&SLUGear2AddGraph, (const void*) 250, TPS_GRID_SIZE * 2);
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
	// 250-291 SLUGear2AddGraph.
	eeprom_update_block((void*)&SLUGear2AddGraph, (void*) 250, TEMP_GRID_SIZE * 2);
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

	// Пихаем все в UART, чтобы потом можно было копипастить.
	send_eeprom_to_uart();
}
