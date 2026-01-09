#include <avr/io.h>			// Названия регистров и номера бит.
#include <stdint.h>			// Коротние название int.
#include <avr/interrupt.h>	// Прерывания.

#include "i2c.h"			// Свой заголовок.

/***********!!! НЕЛЬЗЯ ТРОГАТЬ БУФЕР ПОКА ОН НЕ ОТПРАВЛЕН !!!************/

static volatile uint8_t Status = 0;		// Текущий статус TWI.
static volatile uint8_t Ready = 1;		// Готовность TWI к новому заданию.
static uint8_t *SendBufer;				// Ссылка на внешний массив, первый байт - адрес устройства.
static uint16_t TxMsgSize = 0;			// Количество байт для отправки.
static volatile uint8_t TxBuffPos = 0;	// Позиция в буфере.

#define RX_BUFFER_SIZE 32
static uint8_t ReadMode = 0;			// Режим чтения байт.
static uint8_t ReceiveBuffer[RX_BUFFER_SIZE] = {0};	// Буфер приема.
static uint8_t RxMsgSize = 0;			// Количество байт для приема.
static volatile uint8_t RxBuffPos = 0;	// Позиция в буфере.

// Настройка интерфейса.
void i2c_init() {
	// Порты I2C как вход с подтяжкой (на всякий случай).
	DDRD &= ~(1 << 0);		// PD0
	PORTD |= (1 << 0);		// SCL

	DDRD &= ~(1 << 1);		// PD1
	PORTD |= (1 << 1);		// SDA

	// 72 - 100 кГц, 32 - 200 кГц, 12 - 400 кГц, 8 - 500 кГц.

	TWCR = 0;									// Сброс настроек.
	TWBR = 12;									// Настройка частоты 400 кГц.
	TWSR &= ~((1 << TWPS1) | (1 << TWPS0));		// Предделитель 1.
}

// Приём данных от устройства.
void i2c_receive_data(uint8_t *Array, uint16_t TxSize, uint16_t RxSize) {
	if (!Ready) {return;} 	// Интерфейс занят.

	ReadMode = 1;
	RxBuffPos = 0;
	TxBuffPos = 0;

	SendBufer = Array;		// Сохраняем ссылку на массив
	TxMsgSize = TxSize;		// и размер массива.

	RxMsgSize = RxSize - 1;	// Получаем последний номер байта, который должны получить.

	TWCR = (1 << TWINT)	|
			(1 << TWSTA) |	// Формируем состояние СТАРТ.
			(1 << TWEN) |
			(1 << TWIE);	// Включаем прерывания.

	Ready = 0;				// Начало приёма, TWI занят.
	// Дальше все происходит в прерываниях.
}

uint8_t i2c_get_rx_size() {
	return RxBuffPos;
}

uint8_t i2c_get_rx_byte(uint8_t N) {
	return ReceiveBuffer[N];
}

// Отправка данных, необходимо передать увказатель на массив
// и его размер, первый байт в массиве - это адрес устройства.
void i2c_send_data(uint8_t *Array, uint16_t TxSize) {
	if (!Ready) {return;} 	// Интерфейс занят.

	ReadMode = 0;
	TxBuffPos = 0;

	SendBufer = Array;		// Сохраняем ссылку на массив
	TxMsgSize = TxSize;		// и размер массива.

	TWCR = (1 << TWINT)	|
			(1 << TWSTA) |	// Формируем состояние СТАРТ.
			(1 << TWEN) |		
			(1 << TWIE);	// Включаем прерывания.

	Ready = 0;				// Начало передачи, TWI занят.
	// Дальше все происходит в прерываниях.
}

// Возвращает готовность интерфейса к новому заданию.
uint8_t i2c_ready() {
	cli();
		uint8_t RD = Ready;
	sei();
	return RD;
}

// Возвращает статус интерфейса.
uint8_t i2c_get_status() {
	cli();
		uint8_t CurrentStatus = Status;	// Текущий статус TWI.
	sei();
	return CurrentStatus;
}

//  Обработчик прерывания TWI. 
ISR (TWI_vect) {
  	Status = TWSR & TWSR_MASK;		// Текущий статус TWI.

	switch (Status) {
		case 0x08:							// Передан сигнал START.
			TxBuffPos = 0;					// Сброс позиции в массиве.
			TWDR = (SendBufer[TxBuffPos++] << 1) | 0;			// Записываем адрес в регистр.
			TWCR = (1 << TWINT)	| (1 << TWEN) | (1 << TWIE);	// Отправляем.
			break;
		case 0x10:							// Передан сигнал REPEATED START.
			TxBuffPos = 0;
			// Записываем адрес в регистр.
			TWDR = (SendBufer[TxBuffPos++] << 1) | 1;
			TWCR = (1 << TWINT)	| (1 << TWEN) |	(1 << TWIE);	// Отправляем.
			break;

	// ========== Статусные коды ведущего передатчика ==========

		case 0x18:							// Передан SLA+W, принят ACK.
			TWDR = SendBufer[TxBuffPos++];	// Записываем байт в регистр.
			TWCR = (1 << TWINT)	| (1 << TWEN) | (1 << TWIE); // Отправляем.
			break;
		case 0x28:								// Передан байт данных, принят ACK.
			if (ReadMode) {
				// Формируем состояние СТАРТ повторно.
				TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA) | (1 << TWIE);
			}
			else {
				if (TxBuffPos < TxMsgSize) {
					TWDR = SendBufer[TxBuffPos++];			// Записываем очередной байт в регистр.
					TWCR |= (1 << TWINT) | (1 << TWEN);		// Отправляем.
				}
				else {									// Достигнут конец массива.
					// Формируем состояние СТОП.
					TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
					Status = TWI_SUCCESS;				// Передача завершена успешно.
					Ready = 1;							// Интерфейс свободен.
				}
			}
			break;

	// ========== Статусные коды ведущего приёмника ==========

		case 0x40:							// Передан SLA+R, принят ACK.
			if (RxMsgSize) {		// Формируем состояние ACK (0xC4).
				TWCR = (1 << TWINT)	| (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
			}
			else {					// Формируем состояние NACK (0x84).
				TWCR = (1 << TWINT)	| (1 << TWEN) | (1 << TWIE);
			}
			break;

		case 0x50:								// Принят байт данных, возвращен ACK.
			ReceiveBuffer[RxBuffPos++] = TWDR;	// Записываем байт в массив.
			if (RxBuffPos >= RX_BUFFER_SIZE) {	// Переполнение буфера.
				i2c_stop();
				return;
			}
			if (RxBuffPos >= RxMsgSize) {		// Формируем состояние NACK (0x84).
				TWCR = (1 << TWINT)	| (1 << TWEN) | (1 << TWIE);
			}
			else {								// Формируем состояние ACK (0xС4).
				TWCR = (1 << TWINT)	| (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
			}
			break;

		case 0x58:								// Принят байт данных, возвращен NACK.
			ReceiveBuffer[RxBuffPos++] = TWDR;	// Записываем байт в массив.
			TWCR = (1 << TWINT) |				// Формируем состояние СТОП.
				(1 << TWEN) |
				(1 << TWSTO);
			Status = TWI_SUCCESS;				// Передача завершена успешно.
			Ready = 1;							// Интерфейс свободен.
			break;

	// ========== Коды ошибок ведущего передатчика ==========

		case 0x20:							// Передан SLA+W, принят NACK.
			TWCR = (1 << TWINT) |				// Передача не удалась.
					(1 << TWEN) |
					(1 << TWSTO);				// Формируем состояние СТОП.
			Ready = 1;						// Интерфейс свободен. 	
			break;
		case 0x30:								// Передан байт данных, принят NACK.
			TWCR = (1 << TWINT) |				// Передача не удалась.
					(1 << TWEN) |
					(1 << TWSTO);				// Формируем состояние СТОП.
			Ready = 1;							// Интерфейс свободен. 							
			break;

	// ========== Все прочее пока считаем как ошибку ==========
		default:
			i2c_stop();
			break;
	}
}

void i2c_stop() {
	TWCR = (1 << TWINT) |				// Передача не удалась.
			(1 << TWEN) |
			(1 << TWSTO);				// Формируем состояние СТОП.
	Ready = 1;	// Интерфейс свободен.
}
/*
	При включинии TWI происходит захват управления пинами SCL и SDA.

	Формула расчета частоты CLK:
		F_scl = F_cpu / (16 + 2 * TWBR * 4 ^ TWPS)
	Отсюда значение регистра TWBR рассчитывается так:
		TWBR = ((F_cpu / F_scl) - 16) / (2 * 4 ^ TWPS)
	Пример для 100 кГц
		TWBR = ((16000000 / 100000) - 16) / (2 * 4 ^ 0) = 72.



	https://chipenable.ru/index.php/programming-avr/item/195
	https://microsin.net/programming/avr/ds1307-rtc-clock-with-avr.html

					char buffer [12];
					sprintf(buffer, "0x%x\t0x%x\r\n", Status, TWDR);
					uart_puts(buffer);
*/