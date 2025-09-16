#include <stdint.h>			// Коротние название int.
#include <avr/eeprom.h>		// EEPROM.
#include <avr/interrupt.h>	// Прерывания.

#include "uart.h"			// Свой заголовок.
#include "pinout.h"			// Список назначенных выводов.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "adc.h"			// АЦП.
#include "eeprom.h"			// Чтение и запись EEPROM.
#include "macros.h"			// Макросы.
#include "gears.h"			// Фунции переключения передач.

#include <stdio.h>			// Стандартная библиотека ввода/вывода

// Скорость передачи UART 115200 бит/с.
#define UART_BAUD_RATE 115200UL
// Значения регистров для настройки скорости UART.
// Вычисляется требуемое значение по формуле:
//	UBBR = F_CPU / (16 * baudrate) - 1		для U2X=0
//  UBBR = F_CPU / (8 * baudrate) - 1		для U2X=1

#define SET_UBRR ((F_CPU / (8UL * UART_BAUD_RATE)) - 1UL)

#define UART_RX_BUFFER_SIZE 128						// Размер буфера приема.
uint8_t	ReceiveBuffer[UART_RX_BUFFER_SIZE] = {0};	// Буфер приема.
volatile uint8_t RxBuffPos = 0;						// Позиция в буфере.
// Состояние принятой комманды.
// 0 - ожидание приёма,
// 1 - идёт приём,
// 2 - команда принята.
volatile uint8_t RxCommandStatus = 0;
volatile uint8_t RxMarkerByte = 0;					// Признак, что предыдущий символ был заменен.

#define UART_TX_BUFFER_SIZE 128						// Размер буфера отправки.
uint8_t	SendBuffer[UART_TX_BUFFER_SIZE] = {0};		// Буфер отправки.
uint16_t TxMsgSize = 0;								// Количество байт для отправки.
volatile uint8_t TxBuffPos = 0;						// Позиция в буфере.
volatile uint8_t TxReady = 1;						// UART готов к отправке.

volatile uint8_t UseMarkers = 0;	// Использовать спецсимволы начала/конца пакета.
volatile uint8_t TxMarkerByte = 0;	// Признак, что предыдущий символ был заменен.

char CharArray[8] = {0};

static void send_uint16_array(uint16_t* Array, uint8_t ASize);
static void send_int16_array(int16_t* Array, uint8_t ASize);

static void uart_buffer_add_uint16(uint16_t Value);
static void uart_buffer_add_int16(int16_t Value);

//static void uart_send_table(uint8_t N);
static void uart_write_table(uint8_t N);

static int16_t uart_build_int16(uint8_t i);
static uint16_t uart_build_uint16(uint8_t i);

// Функция программного сброса
void(* resetFunc) (void) = 0;

//eeprom_update_byte (uint8_t *__p, uint8_t __value)

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

void uart_send_tcu_data() {
	if (!TxReady) {return;}		// Не трогать буффер пока идет передача.

	TxBuffPos = 0;	// Сброс позиции.
	UseMarkers = 1;	// Используем байты маркеры.
	SendBuffer[TxBuffPos++] = FOBEGIN;	// Байт начала пакета.
	SendBuffer[TxBuffPos++] = TCU_DATA_PACKET;		// Тип данных.

	//TCU.OilTemp = 78;
	// TCU.SLT = get_adc_value(0); // Температура масла.
	// TCU.SLN = get_adc_value(1); // ДПДЗ.

	// Начальный адрес структуры TCU.
	uint8_t* TCUAddr = (uint8_t*) &TCU;
	// Запихиваем структуру побайтово в массив на отправку.
	for (uint8_t i = 0; i < sizeof(TCU); i++) {
		SendBuffer[TxBuffPos++] = *(TCUAddr + i);
	}
	
	SendBuffer[TxBuffPos++] = FIOEND;		// Байт конца пакета.
	uart_send_array();
}

void uart_send_table(uint8_t N) {
	TxBuffPos = 0;	// Сброс позиции.
	UseMarkers = 1;	// Используем байты маркеры.
	SendBuffer[TxBuffPos++] = FOBEGIN;			// Байт начала пакета.
	SendBuffer[TxBuffPos++] = TCU_TABLE_ANSWER;	// Тип данных - таблица.
	SendBuffer[TxBuffPos++] = N;				// Номер таблицы.

	switch (N) {
		case SLT_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(SLTGraph[i]);}
			break;
		case SLT_TEMP_CORR_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(SLTTempCorrGraph[i]);}
			break;
		case SLN_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(SLNGraph[i]);}
			break;
		case SLU_GEAR2_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(SLUGear2Graph[i]);}
			break;
		case SLU_GEAR2_TEMP_CORR_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(SLUGear2TempCorrGraph[i]);}
			break;
		case SLU_GEAR2_TPS_ADAPT_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_int16(SLUGear2TPSAdaptGraph[i]);}
			break;
		case SLU_GEAR2_TEMP_ADAPT_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(SLUGear2TempAdaptGraph[i]);}
			break;
		case SLU_GEAR2_ADD_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_int16(SLUGear2AddGraph[i]);}
			break;
		case SLU_GEAR3_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(SLUGear3Graph[i]);}
			break;
		case SLU_GEAR3_DELAY_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(SLUGear3DelayGraph[i]);}
			break;
		case SLN_GEAR3_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(SLNGear3Graph[i]);}
			break;
		case SLN_GEAR3_OFFSET_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_int16(SLNGear3OffsetGraph[i]);}
			break;
		default:	// Неверный номер таблицы.
			TxBuffPos = 0;
			UseMarkers = 0;
			return;
	}
	SendBuffer[TxBuffPos++] = FIOEND;		// Байт конца пакета.
	uart_send_array();
}

static void uart_write_table(uint8_t N) {
	switch (N) {
		case SLT_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {SLTGraph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLT_TEMP_CORR_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {SLTTempCorrGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLN_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {SLNGraph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLU_GEAR2_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {SLUGear2Graph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLU_GEAR2_TEMP_CORR_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {SLUGear2TempCorrGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLU_GEAR2_TPS_ADAPT_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {SLUGear2TPSAdaptGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLU_GEAR2_TEMP_ADAPT_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {SLUGear2TempAdaptGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLU_GEAR2_ADD_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {SLUGear2AddGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLU_GEAR3_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {SLUGear3Graph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLU_GEAR3_DELAY_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {SLUGear3DelayGraph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLN_GEAR3_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {SLNGear3Graph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLN_GEAR3_OFFSET_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {SLNGear3OffsetGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		default:	// Неверный номер таблицы.
			return;
	}
	uart_send_table(N);
}

static void uart_buffer_add_uint16(uint16_t Value) {
	uint8_t *pValue = (uint8_t*)&Value;
	SendBuffer[TxBuffPos++] = *pValue;
	SendBuffer[TxBuffPos++] = *(pValue + 1);
}

static void uart_buffer_add_int16(int16_t Value) {
	uint8_t *pValue = (uint8_t*)&Value;
	SendBuffer[TxBuffPos++] = *pValue;
	SendBuffer[TxBuffPos++] = *(pValue + 1);	
}

void uart_command_processing() {
	if (!TxReady) {return;}		// Не трогать буфер пока идет передача.

	if (RxCommandStatus!= 2) {return;}
	if (RxBuffPos < 2) {
		RxCommandStatus = 0;
		return;
	}

	switch (ReceiveBuffer[0]) {
		case GET_TABLE_COMMAND:
			uart_send_table(ReceiveBuffer[1]);
			break;
		case NEW_TABLE_DATA:
			uart_write_table(ReceiveBuffer[1]);
			break;
		case READ_EEPROM_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == READ_EEPROM_COMMAND) {
				read_eeprom();
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case WRITE_EEPROM_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == WRITE_EEPROM_COMMAND) {
				update_eeprom();
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case SPEED_TEST_COMMAND:
			if (SpeedTestFlag) {SpeedTestFlag = 0;}
			else {SpeedTestFlag = 1;}
			break;
		case GEAR_LIMIT_COMMAND:
			if (RxBuffPos == 4) {
				uint8_t Min = ReceiveBuffer[2];
				uint8_t Max = ReceiveBuffer[3];
				if (Min <= Max && Min >= 1 && Min <= 5 && Max >= 1 && Max <= 5) {
					set_gear_limit(Min, Max);
					uart_send_table(ReceiveBuffer[1]);
				}
			}
			break;
		case TABLES_INIT_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == TABLES_INIT_COMMAND) {
			    eeprom_update_byte((uint8_t*) 2048, TABLES_INIT_COMMAND);	// Устанавливаем метку.
				resetFunc();	// Перезапускаем код ЭБУ (переход к нулевому адресу).
			}
			break;

	}
	RxCommandStatus = 0;
}

// Сборка int из двух байт
static int16_t uart_build_int16(uint8_t i) {
	int16_t Value = 0;
	uint8_t *pValue = (uint8_t*)&Value;
	*pValue = ReceiveBuffer[i + 1];
	*(pValue + 1) = ReceiveBuffer[i];
	return Value;
}

// Сборка unsigned int из двух байт
static uint16_t uart_build_uint16(uint8_t i) {
	uint16_t Value = 0;
	uint8_t *pValue = (uint8_t*)&Value;
	*pValue = ReceiveBuffer[i + 1];
	*(pValue + 1) = ReceiveBuffer[i];
	return Value;
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

	uart_send_string("SLNGraph\n");
	send_uint16_array(SLNGraph, TPS_GRID_SIZE);

	uart_send_string("SLUGear2Graph\n");
	send_uint16_array(SLUGear2Graph, TPS_GRID_SIZE);
	uart_send_string("SLUGear2TPSAdaptGraph\n");
	send_int16_array(SLUGear2TPSAdaptGraph, TPS_GRID_SIZE);

	uart_send_string("SLUGear2TempCorrGraph\n");
	send_int16_array(SLUGear2TempCorrGraph, TEMP_GRID_SIZE);
	uart_send_string("SLUGear2TempAdaptGraph\n");
	send_int16_array(SLUGear2TempAdaptGraph, TEMP_GRID_SIZE);

	uart_send_string("SLUGear2AddGraph\n");
	send_int16_array(SLUGear2AddGraph, TPS_GRID_SIZE);

	uart_send_string("SLUGear3Graph\n");
	send_uint16_array(SLUGear3Graph, TPS_GRID_SIZE);
	uart_send_string("SLUGear3DelayGraph\n");
	send_uint16_array(SLUGear3DelayGraph, TPS_GRID_SIZE);
	
	uart_send_string("SLNGear3Graph\n");
	send_uint16_array(SLNGear3Graph, TPS_GRID_SIZE);
	uart_send_string("SLNGear3OffsetGraph\n");
	send_int16_array(SLNGear3OffsetGraph, TPS_GRID_SIZE);

	uart_send_char('\n');
	uart_send_char('\n');
}

void uart_send_uint16(uint16_t N) {
	snprintf(CharArray, 6, "%u", N);
	uart_send_string(CharArray);
}
void uart_send_int16(int16_t N) {
	snprintf(CharArray, 7, "%i", N);
	uart_send_string(CharArray);
}

static void send_uint16_array(uint16_t* Array, uint8_t ASize) {
	for (uint8_t i = 0; i < ASize; i++) {
		snprintf(CharArray, 6, ", %u", Array[i]);
		uart_send_string(CharArray);
	}
	uart_send_char('\n');
}

static void send_int16_array(int16_t* Array, uint8_t ASize) {
	for (uint8_t i = 0; i < ASize; i++) {
		snprintf(CharArray, 7, ", %i", Array[i]);
		uart_send_string(CharArray);
	}
	uart_send_char('\n');
}

// Отправить массив в UART.
void uart_send_array() {
	TxMsgSize = TxBuffPos;		// Количество байт на отправку.
	TxReady = 0;				// UART занят.
	TxBuffPos = 0;				// Сбрасываем позицию в массиве.
	UCSR0B |= (1 << UDRIE0);	// Включаем прерывание по опустошению буфера.
}

// Возвращает готовность интерфейса к новому заданию.
uint8_t uart_tx_ready() {
	cli();
		uint8_t RD = UCSR0A & (1 << UDRE0);
	sei();

	if (RD && TxReady) {return 1;}	// Буфер свободен и не идет пакетная отправка.
	else {return 0;}
}

// Прерывание по опустошению буфера.
ISR (USART0_UDRE_vect) {
	uint8_t SendByte = SendBuffer[TxBuffPos];
	
	if (UseMarkers) {		// Используются маркеры.
		if (TxMarkerByte) {			// Если был маркер.
			SendByte = TxMarkerByte;	// Отправляем символ-замену.
			TxMarkerByte = 0;			// Сбрасываем маркер.
			TxBuffPos++;
		}
		// Первый и последний байт - исключение.
		else if (TxBuffPos > 0 && TxBuffPos < TxMsgSize - 1) {
			switch (SendByte) {
				case FOBEGIN:		// Если байт совпадает с маркером.
					SendByte = FESC;	// Отправляем символ подмены байта
					TxMarkerByte = TFOBEGIN;	// и оставляем маркер.
					break;
				case FIOEND:
					SendByte = FESC;
					TxMarkerByte = TFIOEND;
					break;
				case FESC:
					SendByte = FESC;
					TxMarkerByte = TFESC;
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
	TxReady = 1;
	UseMarkers = 0;
}

// Прерывание по окончании приема.
ISR (USART0_RX_vect) {
	uint8_t OneByte = UDR0;		// Получаем байт.
	if (RxCommandStatus > 1) {return;}

	switch (OneByte) {
		case FOBEGIN:	// Принят начальный байт.
			RxCommandStatus = 1;
			RxBuffPos = 0;
			RxMarkerByte = 0;
			break;
		case FIOEND:	// Принят завершающий байт.
			RxCommandStatus = 2;
			break;
		case FESC:		// Принят символ подмены байта.
			RxMarkerByte = 1;
			break;
		default:
			if (RxBuffPos >= UART_RX_BUFFER_SIZE) {	// Пришло байт больше чем надо.
				RxCommandStatus = 0;
				return;
			}
		 	if (RxMarkerByte) {	// Следующий байт после символа подмены.
		 		switch (OneByte) {
		 			case TFOBEGIN:
		 				OneByte = FOBEGIN;
		 				break;
		 			case TFIOEND:
		 				OneByte = FIOEND;
		 				break;
		 			case TFESC:
		 				OneByte = FESC;
		 				break;
		 			default:		// Если ничего не совпало, значит косяк.
		 				RxCommandStatus = 0;
		 				RxMarkerByte = 0;
		 				return;
		 		}
		 		RxMarkerByte = 0;
		 	}
			ReceiveBuffer[RxBuffPos] = OneByte;
			RxBuffPos++;
			break;
	}
}