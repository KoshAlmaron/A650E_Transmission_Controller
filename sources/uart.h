// UART.

#ifndef _UART_H_
	#define _UART_H_

	void uart_init(uint8_t mode);

	void send_tcu_data();

	uint8_t uart_get_byte();
	void uart_send_char(char Data);
	void uart_send_string(char* s);
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
	
#endif