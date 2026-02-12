// Чтение и запись EEPROM.

#ifndef _EEPROM_H_
	#define _EEPROM_H_

	#define OVERWRITE_BYTE 0xab
	#define OVERWRITE_FIRST_BYTE_NUMBER 3072

	void update_eeprom_adaptation();

	void read_eeprom_tables();
	void update_eeprom_tables();

	void read_eeprom_adc();
	void update_eeprom_adc();

	void read_eeprom_speed();
	void update_eeprom_speed();

	void read_eeprom_config();
	void update_eeprom_config();

	void update_eeprom_add_variables();

#endif