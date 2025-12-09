// UART.

#ifndef _UART_H_
	#define _UART_H_

	void uart_init(uint8_t mode);

	void uart_send_tcu_data();
	void uart_send_cfg_data();
	void uart_send_table(uint8_t N);
	void uart_command_processing();

	void uart_send_array();
	uint8_t uart_tx_ready();

	// Спецсимволы в пакете данных
	#define FOBEGIN	0x40	// '@'  Начало исходящего пакета
	#define FIOEND	0x0D	// '\r' Конец пакета
	#define FESC	0x0A	// '\n' Символ подмены байта (FESC)
	// Измененные байты, которые были в пакете и совпадали со сцецбайтами
	#define TFOBEGIN	0x82	// Измененный FOBEGIN
	#define TFIOEND		0x83	// Измененный FIOEND
	#define TFESC		0x84	// Измененный FESC

	#define TCU_DATA_PACKET 0x71		// Стандартный пакет с данными.

	#define GET_TABLE_COMMAND	0xc1	// Запрос таблицы.
	#define TCU_TABLE_ANSWER	0xc2	// Ответ с таблицей.
	#define NEW_TABLE_DATA		0xc3	// Новые значения для таблицы.

	#define GET_CONFIG_COMMAND	0xc4	// Запрос структуры с конфигурацией.
	#define TCU_CONFIG_ANSWER	0xc5	// Ответ со структурой.
	#define NEW_CONFIG_DATA		0xc6	// Новые значения для структуры конфигурации.

	#define GET_PORTS_STATE		0xc7	// Запрос статуса портов.
	#define PORTS_STATE_PACKET	0xc8	// Ответ со статусами портов.

	#define READ_EEPROM_MAIN_COMMAND	0xe0	// Считать EEPROM - Таблицы.
	#define READ_EEPROM_ADC_COMMAND		0xe1	// Считать EEPROM - АЦП.
	#define READ_EEPROM_SPEED_COMMAND	0xe2	// Считать EEPROM - Скорость.
	#define READ_EEPROM_CONFIG_COMMAND	0xe3	// Считать EEPROM - Настройки.

	#define WRITE_EEPROM_MAIN_COMMAND	0xea	// Записать EEPROM - Таблицы.
	#define WRITE_EEPROM_ADC_COMMAND	0xeb	// Записать EEPROM - АЦП.
	#define WRITE_EEPROM_SPEED_COMMAND	0xec	// Записать EEPROM - Скорость.
	#define WRITE_EEPROM_CONFIG_COMMAND	0xed	// Записать EEPROM - Настройки.

	#define SPEED_TEST_COMMAND			0xd0	// Переключить режим тестирования спидометра.
	#define GEAR_LIMIT_COMMAND			0xd1	// Установить ограничения передачь.

	#define TABLES_INIT_MAIN_COMMAND	0xda	// Записать в ОЗУ значения из прошивки - Таблицы.
	#define TABLES_INIT_ADC_COMMAND		0xdb	// Записать в ОЗУ значения из прошивки - АЦП.
	#define TABLES_INIT_SPEED_COMMAND	0xdc	// Записать в ОЗУ значения из прошивки - Скорость.
	#define TABLES_INIT_CONFIG_COMMAND	0xdd	// Записать в ОЗУ значения из прошивки - Настройки.

	#define APPLY_G2_TPS_ADAPT_COMMAND		0xf0	// Применить адаптацию второй передачи по ДПДЗ.
	#define APPLY_G2_TEMP_ADAPT_COMMAND		0xf1	// Применить адаптацию второй передачи по температуре.
	#define APPLY_G2_ADV_ADAPT_COMMAND		0xf2	// Применить адаптацию реактивации второй передачи по ДПДЗ.
	#define APPLY_G2_ADV_TEMP_ADAPT_COMMAND	0xf3	// Применить адаптацию реактивации второй передачи по температуре.

	#define APPLY_G3_TPS_ADAPT_COMMAND	0xf4	// Применить адаптацию третьей передачи по ДПДЗ.
	#define APPLY_G3_TEMP_ADAPT_COMMAND	0xf5	// Применить адаптацию третьей передачи по температуре.

#endif