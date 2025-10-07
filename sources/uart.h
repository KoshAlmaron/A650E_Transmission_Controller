// UART.

#ifndef _UART_H_
	#define _UART_H_

	void uart_init(uint8_t mode);

	void uart_send_tcu_data();
	void uart_send_table(uint8_t N);
	void uart_command_processing();

	void uart_send_char(char Data);
	void uart_send_string(char* s);

	void uart_send_uint16(uint16_t N);
	void uart_send_int16(int16_t N);
	
	void send_eeprom_to_uart();

	void uart_send_array();
	uint8_t uart_tx_ready();

	// Спецсимволы в пакете данных
	#define FOBEGIN  0x40       // '@'  Начало исходящего пакета
	#define FIOEND   0x0D       // '\r' Конец пакета
	#define FESC     0x0A       // '\n' Символ подмены байта (FESC)
	// Измененные байты, которые были в пакете и совпадали со сцецбайтами
	#define TFOBEGIN 0x82       // Измененный FOBEGIN
	#define TFIOEND  0x83       // Измененный FIOEND
	#define TFESC    0x84       // Измененный FESC

	#define TCU_DATA_PACKET 0x71		// Стандартный пакет с данными.

	#define GET_TABLE_COMMAND 0xc1		// Запрос таблицы.
	#define TCU_TABLE_ANSWER 0xc2		// Ответ с таблицей.
	#define NEW_TABLE_DATA 0xc8			// Новые значения для таблицы.

	#define READ_EEPROM_COMMAND 0xcc	// Считать EEPROM.
	#define WRITE_EEPROM_COMMAND 0xee	// Записать  EEPROM.

	#define SPEED_TEST_COMMAND 0xde		// Переключить режим тестирования спидометра.
	#define GEAR_LIMIT_COMMAND 0xbe		// Установить ограничения передачь.
	#define TABLES_INIT_COMMAND 0xab	// Записать в таблицы значения из прошивки.

	#define APPLY_G2_TPS_ADAPT_COMMAND 0xfc		// Применить адаптацию второй передачи по ДПДЗ.
	#define APPLY_G2_TEMP_ADAPT_COMMAND 0xfd	// Применить адаптацию второй передачи по температуре.
	
#endif


/*
0x71 - Стандартный пакет с параметрами ЭБУ
0x80 - Запрос таблицы из ЭБУ
0x81 - Ответ ЭБУ с таблицей
0x90 - Таблица для ЭБУ
0xee - Комманда на запись параметров в EEPROM


*/