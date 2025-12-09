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
#include "selector.h"		// Положение селектора АКПП.
#include "configuration.h"	// Настройки.

#include <stdio.h>			// Стандартная библиотека ввода/вывода

// Скорость передачи UART 115200 бит/с.
#define UART_BAUD_RATE 115200UL
// Значения регистров для настройки скорости UART.
// Вычисляется требуемое значение по формуле:
//	UBBR = F_CPU / (16 * baudrate) - 1		для U2X=0
//  UBBR = F_CPU / (8 * baudrate) - 1		для U2X=1

#define SET_UBRR ((F_CPU / (8UL * UART_BAUD_RATE)) - 1UL)

#define UART_RX_BUFFER_SIZE 200						// Размер буфера приема.
uint8_t	ReceiveBuffer[UART_RX_BUFFER_SIZE] = {0};	// Буфер приема.
volatile uint8_t RxBuffPos = 0;						// Позиция в буфере.
// Состояние принятой комманды.
// 0 - ожидание приёма,
// 1 - идёт приём,
// 2 - команда принята.
volatile uint8_t RxCommandStatus = 0;
volatile uint8_t RxMarkerByte = 0;					// Признак, что предыдущий символ был заменен.

#define UART_TX_BUFFER_SIZE 200						// Размер буфера отправки.
uint8_t	SendBuffer[UART_TX_BUFFER_SIZE] = {0};		// Буфер отправки.
uint16_t TxMsgSize = 0;								// Количество байт для отправки.
volatile uint8_t TxBuffPos = 0;						// Позиция в буфере.
volatile uint8_t TxReady = 1;						// UART готов к отправке.

volatile uint8_t UseMarkers = 0;	// Использовать спецсимволы начала/конца пакета.
volatile uint8_t TxMarkerByte = 0;	// Признак, что предыдущий символ был заменен.

char CharArray[8] = {0};

uint8_t SendPortsStateCount = 0;		// Флаг-счетчик отправки пакета с портами вместо стандартного.

static void uart_buffer_add_uint8(uint8_t Value);

static void uart_buffer_add_uint16(uint16_t Value);
static void uart_buffer_add_int16(int16_t Value);

static void uart_write_table(uint8_t N);

static int16_t uart_build_int16(uint8_t i);
static uint16_t uart_build_uint16(uint8_t i);

static void uart_write_cfg_data();

static void uart_send_ports_state();

// Функция программного сброса
void(* resetFunc) (void) = 0;

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

	if (SendPortsStateCount) {
		SendPortsStateCount--;
		uart_send_ports_state();
		return;
	}

	TxBuffPos = 0;	// Сброс позиции.
	UseMarkers = 1;	// Используем байты маркеры.
	SendBuffer[TxBuffPos++] = FOBEGIN;			// Байт начала пакета.
	SendBuffer[TxBuffPos++] = TCU_DATA_PACKET;	// Тип данных.

	//TCU.GearChangeSLU = 0;

	// Начальный адрес структуры TCU.
	uint8_t* TCUAddr = (uint8_t*) &TCU;
	// Запихиваем структуру побайтово в массив на отправку.
	for (uint8_t i = 0; i < sizeof(TCU); i++) {
		SendBuffer[TxBuffPos++] = *(TCUAddr + i);
	}

	SendBuffer[TxBuffPos++] = FIOEND;		// Байт конца пакета.
	uart_send_array();
}

void uart_send_cfg_data() {
	TxBuffPos = 0;				// Сброс позиции.
	UseMarkers = 1;				// Используем байты маркеры.
	SendBuffer[TxBuffPos++] = FOBEGIN;				// Байт начала пакета.
	SendBuffer[TxBuffPos++] = TCU_CONFIG_ANSWER;	// Тип данных.

	// Начальный адрес структуры TCU.
	uint8_t* CFGAddr = (uint8_t*) &CFG;
	// Запихиваем структуру побайтово в массив на отправку.
	for (uint8_t i = 0; i < sizeof(CFG); i++) {
		SendBuffer[TxBuffPos++] = *(CFGAddr + i);
	}

	SendBuffer[TxBuffPos++] = FIOEND;		// Байт конца пакета.
	uart_send_array();
}

static void uart_write_cfg_data() {
	if (RxBuffPos != sizeof(CFG) + 2) {return;}
	
	// Начальный адрес структуры TCU.
	uint8_t* CFGAddr = (uint8_t*) &CFG;
	// Запихиваем буфер обратно в структуру побайтово.
	for (uint8_t i = 0; i < sizeof(CFG); i++) {
		*(CFGAddr + i) = ReceiveBuffer[i + 2];
	}

	uart_send_cfg_data();
}

void uart_send_table(uint8_t N) {
	TxBuffPos = 0;	// Сброс позиции.
	UseMarkers = 1;	// Используем байты маркеры.
	SendBuffer[TxBuffPos++] = FOBEGIN;			// Байт начала пакета.
	SendBuffer[TxBuffPos++] = TCU_TABLE_ANSWER;	// Тип данных - таблица.
	SendBuffer[TxBuffPos++] = N;				// Номер таблицы.

	switch (N) {
		case SLT_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(TABLES.SLTGraph[i]);}
			break;
		case SLT_TEMP_CORR_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(TABLES.SLTTempCorrGraph[i]);}
			break;
		case SLN_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(TABLES.SLNGraph[i]);}
			break;
		case SLN_TEMP_CORR_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(TABLES.SLNTempCorrGraph[i]);}
			break;
		case SLU_GEAR2_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(TABLES.SLUGear2Graph[i]);}
			break;
		case SLU_GEAR2_TEMP_CORR_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(TABLES.SLUGear2TempCorrGraph[i]);}
			break;
		case SLU_GEAR2_TPS_ADAPT_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_int16(ADAPT.SLUGear2TPSAdaptGraph[i]);}
			break;
		case SLU_GEAR2_TEMP_ADAPT_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(ADAPT.SLUGear2TempAdaptGraph[i]);}
			break;
		case GEAR_CHANGE_STEP_ARRAY:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(TABLES.GearChangeStepArray[i]);}
			break;
		case GEAR2_ADV_GRAPH:
			for (uint8_t i = 0; i < DELTA_RPM_GRID_SIZE; i++) {uart_buffer_add_int16(TABLES.Gear2AdvGraph[i]);}
			break;
		case GEAR2_ADV_ADAPT_GRAPH:
			for (uint8_t i = 0; i < DELTA_RPM_GRID_SIZE; i++) {uart_buffer_add_int16(ADAPT.Gear2AdvAdaptGraph[i]);}
			break;
		case GEAR2_ADV_TEMP_CORR_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(TABLES.Gear2AdvTempCorrGraph[i]);}
			break;
		case GEAR2_ADV_TEMP_ADAPT_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(ADAPT.Gear2AdvTempAdaptGraph[i]);}
			break;
		case SLU_GEAR3_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(TABLES.SLUGear3Graph[i]);}
			break;
		case SLU_GEAR3_DELAY_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(TABLES.SLUGear3DelayGraph[i]);}
			break;
		case SLU_G3_DELAY_TEMP_CORR_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(TABLES.SLUG3DelayTempCorrGraph[i]);}
			break;
		case SLU_GEAR3_TPS_ADAPT_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_int16(ADAPT.SLUGear3TPSAdaptGraph[i]);}
			break;
		case SLU_GEAR3_TEMP_ADAPT_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(ADAPT.SLUGear3TempAdaptGraph[i]);}
			break;
		case SLN_GEAR3_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_uint16(TABLES.SLNGear3Graph[i]);}
			break;
		case SLN_GEAR3_OFFSET_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_int16(TABLES.SLNGear3OffsetGraph[i]);}
			break;
		case TPS_ADC_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {uart_buffer_add_int16(ADCTBL.TPSGraph[i]);}
			break;
		case OIL_ADC_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {uart_buffer_add_int16(ADCTBL.OilTempGraph[i]);}
			break;
		case GEAR_SPEED_GRAPHS:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {
				uart_buffer_add_uint8(SPEED.Gear_2_1[i]);
				uart_buffer_add_uint8(SPEED.Gear_1_2[i]);
				uart_buffer_add_uint8(SPEED.Gear_3_2[i]);
				uart_buffer_add_uint8(SPEED.Gear_2_3[i]);
				uart_buffer_add_uint8(SPEED.Gear_4_3[i]);
				uart_buffer_add_uint8(SPEED.Gear_3_4[i]);
				uart_buffer_add_uint8(SPEED.Gear_5_4[i]);
				uart_buffer_add_uint8(SPEED.Gear_4_5[i]);
			}
			break;

		default:	// Неверный номер таблицы.
			TxBuffPos = 0;
			UseMarkers = 0;
			return;
	}
	SendBuffer[TxBuffPos++] = FIOEND;		// Байт конца пакета.
	uart_send_array();
}

static void uart_send_ports_state() {
	if (!TxReady) {return;}

	TxBuffPos = 0;
	UseMarkers = 1;

	SendBuffer[TxBuffPos++] = FOBEGIN;
	SendBuffer[TxBuffPos++] = PORTS_STATE_PACKET;

	// Последовательно отправляем PINx и DDRx для всех портов ATmega2560.
	SendBuffer[TxBuffPos++] = PINA;
	SendBuffer[TxBuffPos++] = DDRA;

	SendBuffer[TxBuffPos++] = PINB;
	SendBuffer[TxBuffPos++] = DDRB;

	SendBuffer[TxBuffPos++] = PINC;
	SendBuffer[TxBuffPos++] = DDRC;

	SendBuffer[TxBuffPos++] = PIND;
	SendBuffer[TxBuffPos++] = DDRD;

	SendBuffer[TxBuffPos++] = PINE;
	SendBuffer[TxBuffPos++] = DDRE;

	SendBuffer[TxBuffPos++] = PINF;
	SendBuffer[TxBuffPos++] = DDRF;

	SendBuffer[TxBuffPos++] = PING;
	SendBuffer[TxBuffPos++] = DDRG;

	SendBuffer[TxBuffPos++] = PINH;
	SendBuffer[TxBuffPos++] = DDRH;

	SendBuffer[TxBuffPos++] = PINJ;
	SendBuffer[TxBuffPos++] = DDRJ;

	SendBuffer[TxBuffPos++] = PINK;
	SendBuffer[TxBuffPos++] = DDRK;

	SendBuffer[TxBuffPos++] = PINL;
	SendBuffer[TxBuffPos++] = DDRL;

	// Дополнительный байт состояния селектора.
	SendBuffer[TxBuffPos++] = get_selector_byte();

	SendBuffer[TxBuffPos++] = FIOEND;
	uart_send_array();
}

void uart_command_processing() {
	if (!TxReady) {return;}		// Не трогать буфер пока идет передача.

	if (RxCommandStatus != 2) {return;}

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
		case GET_CONFIG_COMMAND:
			uart_send_cfg_data();
			break;
		case NEW_CONFIG_DATA:
			uart_write_cfg_data();
			break;
		case GET_PORTS_STATE:
			SendPortsStateCount = SEND_PORT_STATE_COUNT;
			break;
		case READ_EEPROM_MAIN_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == READ_EEPROM_MAIN_COMMAND) {
				read_eeprom_tables();
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case READ_EEPROM_ADC_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == READ_EEPROM_ADC_COMMAND) {
				read_eeprom_adc();
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case READ_EEPROM_SPEED_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == READ_EEPROM_SPEED_COMMAND) {
				read_eeprom_speed();
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case READ_EEPROM_CONFIG_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == READ_EEPROM_CONFIG_COMMAND) {
				read_eeprom_config();
				uart_send_cfg_data();
			}
			break;
		case WRITE_EEPROM_MAIN_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == WRITE_EEPROM_MAIN_COMMAND) {
				update_eeprom_tables();
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case WRITE_EEPROM_ADC_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == WRITE_EEPROM_ADC_COMMAND) {
				update_eeprom_adc();
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case WRITE_EEPROM_SPEED_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == WRITE_EEPROM_SPEED_COMMAND) {
				update_eeprom_speed();
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case WRITE_EEPROM_CONFIG_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == WRITE_EEPROM_CONFIG_COMMAND) {
				update_eeprom_config();
				uart_send_cfg_data();
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
		case TABLES_INIT_MAIN_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == TABLES_INIT_MAIN_COMMAND) {
				eeprom_update_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 0, OVERWRITE_BYTE);	// Устанавливаем метку.
				resetFunc();	// Перезапускаем код ЭБУ (переход к нулевому адресу).
			}
			break;
		case TABLES_INIT_ADC_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == TABLES_INIT_ADC_COMMAND) {
				eeprom_update_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 1, OVERWRITE_BYTE);	// Устанавливаем метку.
				resetFunc();	// Перезапускаем код ЭБУ (переход к нулевому адресу).
			}
			break;
		case TABLES_INIT_SPEED_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == TABLES_INIT_SPEED_COMMAND) {
				eeprom_update_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 2, OVERWRITE_BYTE);	// Устанавливаем метку.
				resetFunc();	// Перезапускаем код ЭБУ (переход к нулевому адресу).
			}
			break;
		case TABLES_INIT_CONFIG_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == TABLES_INIT_CONFIG_COMMAND) {
				eeprom_update_byte((uint8_t*) OVERWRITE_FIRST_BYTE_NUMBER + 3, OVERWRITE_BYTE);	// Устанавливаем метку.
				resetFunc();	// Перезапускаем код ЭБУ (переход к нулевому адресу).
			}
			break;
		case APPLY_G2_TPS_ADAPT_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == APPLY_G2_TPS_ADAPT_COMMAND) {
				for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {
					TABLES.SLUGear2Graph[i] += ADAPT.SLUGear2TPSAdaptGraph[i];
					ADAPT.SLUGear2TPSAdaptGraph[i] = 0;
				}
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case APPLY_G2_TEMP_ADAPT_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == APPLY_G2_TEMP_ADAPT_COMMAND) {
				for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {
					TABLES.SLUGear2TempCorrGraph[i] += ADAPT.SLUGear2TempAdaptGraph[i];
					ADAPT.SLUGear2TempAdaptGraph[i] = 0;
				}
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case APPLY_G2_ADV_ADAPT_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == APPLY_G2_ADV_ADAPT_COMMAND) {
				for (uint8_t i = 0; i < DELTA_RPM_GRID_SIZE; i++) {
					TABLES.Gear2AdvGraph[i] += ADAPT.Gear2AdvAdaptGraph[i];
					ADAPT.Gear2AdvAdaptGraph[i] = 0;
				}
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case APPLY_G2_ADV_TEMP_ADAPT_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == APPLY_G2_ADV_TEMP_ADAPT_COMMAND) {
				for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {
					TABLES.Gear2AdvTempCorrGraph[i] += ADAPT.Gear2AdvTempAdaptGraph[i];
					ADAPT.Gear2AdvTempAdaptGraph[i] = 0;
				}
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case APPLY_G3_TPS_ADAPT_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == APPLY_G3_TPS_ADAPT_COMMAND) {
				for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {
					TABLES.SLUGear3DelayGraph[i] += ADAPT.SLUGear3TPSAdaptGraph[i];
					ADAPT.SLUGear3TPSAdaptGraph[i] = 0;
				}
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
		case APPLY_G3_TEMP_ADAPT_COMMAND:
			if (RxBuffPos == 3 && ReceiveBuffer[2] == APPLY_G3_TEMP_ADAPT_COMMAND) {
				for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {
					TABLES.SLUG3DelayTempCorrGraph[i] += ADAPT.SLUGear3TempAdaptGraph[i];
					ADAPT.SLUGear3TempAdaptGraph[i] = 0;
				}
				uart_send_table(ReceiveBuffer[1]);
			}
			break;
	}
	RxCommandStatus = 0;
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

static void uart_write_table(uint8_t N) {
	switch (N) {
		case SLT_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {TABLES.SLTGraph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLT_TEMP_CORR_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {TABLES.SLTTempCorrGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLN_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {TABLES.SLNGraph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLN_TEMP_CORR_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {TABLES.SLNTempCorrGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLU_GEAR2_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {TABLES.SLUGear2Graph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLU_GEAR2_TEMP_CORR_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {TABLES.SLUGear2TempCorrGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLU_GEAR2_TPS_ADAPT_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {ADAPT.SLUGear2TPSAdaptGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLU_GEAR2_TEMP_ADAPT_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {ADAPT.SLUGear2TempAdaptGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case GEAR_CHANGE_STEP_ARRAY:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {TABLES.GearChangeStepArray[i] = uart_build_int16(2 + i * 2);}
			break;
		case GEAR2_ADV_GRAPH:
			if (RxBuffPos != DELTA_RPM_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < DELTA_RPM_GRID_SIZE; i++) {TABLES.Gear2AdvGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case GEAR2_ADV_ADAPT_GRAPH:
			if (RxBuffPos != DELTA_RPM_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < DELTA_RPM_GRID_SIZE; i++) {ADAPT.Gear2AdvAdaptGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case GEAR2_ADV_TEMP_CORR_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {TABLES.Gear2AdvTempCorrGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case GEAR2_ADV_TEMP_ADAPT_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {ADAPT.Gear2AdvTempAdaptGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLU_GEAR3_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {TABLES.SLUGear3Graph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLU_GEAR3_DELAY_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {TABLES.SLUGear3DelayGraph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLU_G3_DELAY_TEMP_CORR_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {TABLES.SLUG3DelayTempCorrGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLU_GEAR3_TPS_ADAPT_GRAPH:
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {ADAPT.SLUGear3TPSAdaptGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLU_GEAR3_TEMP_ADAPT_GRAPH:
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {ADAPT.SLUGear3TempAdaptGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case SLN_GEAR3_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {TABLES.SLNGear3Graph[i] = uart_build_uint16(2 + i * 2);}
			break;
		case SLN_GEAR3_OFFSET_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {TABLES.SLNGear3OffsetGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case TPS_ADC_GRAPH:
			if (RxBuffPos != TPS_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {ADCTBL.TPSGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case OIL_ADC_GRAPH:
			if (RxBuffPos != TEMP_GRID_SIZE * 2 + 2) {return;}
			for (uint8_t i = 0; i < TEMP_GRID_SIZE; i++) {ADCTBL.OilTempGraph[i] = uart_build_int16(2 + i * 2);}
			break;
		case GEAR_SPEED_GRAPHS:
			if (RxBuffPos != TPS_GRID_SIZE * 8 + 2) {return;}
			for (uint8_t i = 0; i < TPS_GRID_SIZE; i++) {
				SPEED.Gear_2_1[i] = ReceiveBuffer[2 + i * 8 + 0];
				SPEED.Gear_1_2[i] = ReceiveBuffer[2 + i * 8 + 1];
				SPEED.Gear_3_2[i] = ReceiveBuffer[2 + i * 8 + 2];
				SPEED.Gear_2_3[i] = ReceiveBuffer[2 + i * 8 + 3];
				SPEED.Gear_4_3[i] = ReceiveBuffer[2 + i * 8 + 4];
				SPEED.Gear_3_4[i] = ReceiveBuffer[2 + i * 8 + 5];
				SPEED.Gear_5_4[i] = ReceiveBuffer[2 + i * 8 + 6];
				SPEED.Gear_4_5[i] = ReceiveBuffer[2 + i * 8 + 7];
			}
			break;

		default:	// Неверный номер таблицы.
			return;
	}
	uart_send_table(N);
}

static void uart_buffer_add_uint8(uint8_t Value) {
	SendBuffer[TxBuffPos++] = Value;
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