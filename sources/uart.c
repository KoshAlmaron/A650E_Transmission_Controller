#include <stdint.h>			// Коротние название int.
#include <avr/interrupt.h>	// Прерывания.

#include "uart.h"			// Свой заголовок.
#include "pinout.h"			// Список назначенных выводов.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include <stdio.h>			// Стандартная библиотека ввода/вывода

// Скорость передачи UART 57600 бит/с.
#define UART_BAUD_RATE 115200UL
// Значения регистров для настройки скорости UART.
// Вычисляется требуемое значение по формуле:
//	UBBR = F_CPU / (16 * baudrate) - 1		для U2X=0
//  UBBR = F_CPU / (8 * baudrate) - 1		для U2X=1

#define SET_UBRR ((F_CPU / (8UL * UART_BAUD_RATE)) - 1UL)


#define UART_TX_BUFFER_SIZE 64					// Размер буфера отправки.
uint8_t	SendBuffer[UART_TX_BUFFER_SIZE] = {0};	// Буфер отправки.
uint16_t TxMsgSize = 0;								// Количество байт для отправки.
volatile uint16_t TxBuffPos = 0;					// Позиция в буфере.
volatile uint8_t TXReady = 1;						// UART готов к отправке.

// Использовать спецсимволы начала/конца пакета.
volatile uint8_t UseMarkers = 0;
// Признак, что предыдущий символ был заменен.
volatile uint8_t MarkerByte = 0;

static void send_uint16_array(uint16_t* Array, uint8_t ASize);
static void send_int16_array(int16_t* Array, uint8_t ASize);

void uart_init(uint8_t mode) {
	// Сброс регистров настроек, так как загрузчик Arduino может нагадить.
	UCSR0A = 0;
	UCSR0B = 0;
	UCSR0C = 0;

	if (mode) {
		UCSR0A |= (1 << U2X0);							// Двойная скорость передачи.
		UCSR0C &= ~((1 << UMSEL01) | (1 << UMSEL00));	// Асинхронный режим.
		UCSR0C |= (1 << UCSZ00) | ( 1 << UCSZ01);		// Размер пакета 8 бит.
		UBRR0H = (uint8_t) (SET_UBRR >> 8);				// Настройка скорости.
		UBRR0L = (uint8_t) SET_UBRR;
	}
	else {
		// 0 - uart выключен.
		return;
	}
	
	switch (mode) {
		case 1:			
			UCSR0B |= (1 << RXEN0);		// 1 - Только прием.
			UCSR0B |= (1 << RXCIE0);		// Прерывание по завершеию приёма.
			break;
		case 2:
			UCSR0B |= (1 << TXEN0);		// 2 - Только передача.
			UCSR0B |= (1 << TXCIE0);	// Прерывание по завершеию передачи.
			break;
		case 3:
			// 3 - прием / передача.
			UCSR0B |= (1 << TXEN0);		// Прием.
			UCSR0B |= (1 << RXEN0);		// Передача.
			UCSR0B |= (1 << RXCIE0);	// Прерывание по завершеию приёма.
			UCSR0B |= (1 << TXCIE0);	// Прерывание по завершеию передачи.
			break;
	}
}

void send_tcu_data() {
	TxBuffPos = 0;	// Сброс позиции.
	UseMarkers = 1;	// Используем байты маркеры.
	SendBuffer[TxBuffPos++] = FOBEGIN;	// Байт начала пакета.

	//TCU.OilTemp = 78;

	// Начальный адрес структуры TCU.
	uint8_t* TCUAddr = (uint8_t*) &TCU;
	// Запихиваем структуру побайтово в массив на отправку.
	for (uint8_t i = 0; i < sizeof(TCU); i++) {
		SendBuffer[TxBuffPos++] = *(TCUAddr + i);
	}
	
	SendBuffer[TxBuffPos++] = FIOEND;		// Байт конца пакета.
	uart_send_array();
}

// Принять байт из UART.
uint8_t uart_get_byte() {
	// Ожидание приёма.
	while (!(UCSR0A & (1 << RXC0)));
	// Вернуть принятый байт.
	return UDR0;                    
}

// Отправить символ в UART
void uart_send_char(char Data) {
	// Ожидание готовности.
	while (!(UCSR0A & (1 << UDRE0))); 
	// Отправка.
	UDR0 = Data;                    
}

// Отправить строку в UART, используется только для отладки.
void uart_send_string(char* s) {
	// Пока строка не закончилась.
	while (*s) {
		// Отправить очередной символ.
		uart_send_char(*s);
		// Передвинуть указатель на следующий символ.
		s++;            
	}
}

void send_eeprom_to_uart() {
	while (!uart_tx_ready());	// Ждем окончани предыдущей передачи.

	uart_send_char('\n');
	uart_send_char('\n');

	uart_send_string("SLTGraph\n");
	send_uint16_array(SLTGraph, TPS_GRID_SIZE);

	uart_send_string("SLTTempCorrGraph\n");
	send_int16_array(SLTTempCorrGraph, TEMP_GRID_SIZE);

	uart_send_string("SLUGear2Graph\n");
	send_uint16_array(SLUGear2Graph, TPS_GRID_SIZE);

	uart_send_string("SLUGear3DelayGraph\n");
	send_uint16_array(SLUGear3DelayGraph, TPS_GRID_SIZE);

	uart_send_string("SLUGear2TempCorrGraph\n");
	send_int16_array(SLUGear2TempCorrGraph, TEMP_GRID_SIZE);

	uart_send_string("SLNGraph\n");
	send_uint16_array(SLNGraph, TPS_GRID_SIZE);

	uart_send_string("Gear2DeltaRPM\n");
	send_uint16_array(Gear2DeltaRPM, TPS_GRID_SIZE);
	
	uart_send_string("SLUGear2TPSAdaptGraph\n");
	send_int16_array(SLUGear2TPSAdaptGraph, TPS_GRID_SIZE);

	uart_send_string("SLUGear2TempAdaptGraph\n");
	send_int16_array(SLUGear2TempAdaptGraph, TEMP_GRID_SIZE);

	uart_send_char('\n');
	uart_send_char('\n');
}

static void send_uint16_array(uint16_t* Array, uint8_t ASize) {
	char Chararray[8] = {0};

	snprintf(Chararray, 4, "%3u", Array[0]);
	uart_send_string(Chararray);

	for (uint8_t i = 1; i < ASize; i++) {
		snprintf(Chararray, 6, ", %3u", Array[i]);
		uart_send_string(Chararray);
	}
	uart_send_char('\n');
}

static void send_int16_array(int16_t* Array, uint8_t ASize) {
	char Chararray[8] = {0};

	snprintf(Chararray, 4, "%3i", Array[0]);
	uart_send_string(Chararray);

	for (uint8_t i = 1; i < ASize; i++) {
		snprintf(Chararray, 6, ", %3i", Array[i]);
		uart_send_string(Chararray);
	}
	uart_send_char('\n');
}

// Отправить массив в UART.
void uart_send_array() {
	TxMsgSize = TxBuffPos;		// Количество байт на отправку.
	TXReady = 0;				// UART занят.
	TxBuffPos = 0;				// Сбрасываем позицию в массиве.
	UCSR0B |= (1 << UDRIE0);	// Включаем прерывание по опустошению буфера.
}

// Возвращает готовность интерфейса к новому заданию.
uint8_t uart_tx_ready() {
	cli();
		uint8_t RD = UCSR0A & (1 << UDRE0);
	sei();

	if (RD && TXReady) {return 1;}	// Буфер свободен и не идет пакетная отправка.
	else {return 0;}
}

// Прерывание по опустошению буфера.
ISR (USART0_UDRE_vect) {
	uint8_t SendByte = SendBuffer[TxBuffPos];
	
	if (UseMarkers) {		// Используются маркеры.
		if (MarkerByte) {			// Если был маркер.
			SendByte = MarkerByte;	// Отправляем символ-замену.
			MarkerByte = 0;			// Сбрасываем маркер.
			TxBuffPos++;
		}
		// Первый и последний байт - исключение.
		else if (TxBuffPos > 0 && TxBuffPos < TxMsgSize - 1) {
			switch (SendByte) {
				case FOBEGIN:		// Если байт совпадает с маркером.
					SendByte = FESC;	// Отправляем символ подмены байта
					MarkerByte = TFOBEGIN;	// и оставляем маркер.
					break;
				case FIOEND:
					SendByte = FESC;
					MarkerByte = TFIOEND;
					break;
				case FESC:
					SendByte = FESC;
					MarkerByte = TFESC;
					break;
				default:
					TxBuffPos++;	// Если нет совпадения, переходим к следующему байту.
			}
		}
		else {TxBuffPos++;}
	}
	else {TxBuffPos++;}

	UDR0 = SendByte;	// Загружаем очередной байт.
	if (TxBuffPos >= TxMsgSize) {		// Все данные загружены в буфер отправки.
		UCSR0B &=~ (1 << UDRIE0);	// Запрещаем прерывание.
	}
}

// Прерывание по окончании передачи.
ISR (USART0_TX_vect) {
	TXReady = 1;
	UseMarkers = 0;
}