#include <avr/eeprom.h>
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "eeprom.h"			// Свой заголовок.

// Чтение EEPROM
void read_eeprom() {
					 // Адрес массива	  адрес ячейки	   кол-во.	
	eeprom_read_block((void*)&SLUB3Graph, (const void*) 0, 14);		// 0-13 - SLUB3Graph.
	eeprom_read_block((void*)&SLUB2Graph, (const void*) 14, 14);	// 14-27 - SLUB2Graph.
	SLTB2Add = eeprom_read_byte((uint8_t*) 28);						// 28 - SLTB2Add
}

// Запись EEPROM
void update_eeprom() {
	eeprom_update_block((void*)&SLUB3Graph, (void*) 0, 14);			// 0-13 - SLUB3Graph.
	eeprom_update_block((void*)&SLUB2Graph, (void*) 14, 14);		// 14-27 - SLUB2Graph.
	eeprom_write_byte((uint8_t*) 28, SLTB2Add);				// 28 - SLTB2Add
}