#include <avr/io.h>			// Названия регистров и номера бит.
#include <stdint.h>			// Коротние название int.
#include <avr/interrupt.h>	// Прерывания.
#include "i2c.h"			// Свой заголовок.

#include "uart.h"			// UART.

/***********!!! НЕЛЬЗЯ ТРОГАТЬ БУФЕР ПОКА ОН НЕ ОТПРАВЛЕН !!!************/

volatile uint8_t Status = 0;	// Текущий статус TWI.
volatile uint8_t Ready = 1;		// Готовность TWI к новому заданию.
uint8_t *SendBufer;				// Ссылка на внешний массив, первый байт - адрес устройства.
uint16_t BuferSize;				// Размер внешнего массива.

// Настройка интерфейса.
void i2c_init() {
	// Порты I2C как вход с подтяжкой (на всякий случай).
	DDRD &= ~(1 << 1);		// PD1
	PORTD |= (1 << 1);		// SDA

	DDRD &= ~(1 << 0);		// PD0
	PORTD |= (1 << 0);		// SCL

	// 72 - 100 кГц, 32 - 200 кГц, 12 - 400 кГц.

	TWCR = 0;									// Сброс настроек.
	TWBR = 12;									// Настройка частоты 100 кГц.
	TWSR &= ~((1 << TWPS1) | (1 << TWPS0));		// Предделитель 1.
}

// Отправка данных, необходимо передать увказатель на массив
// и его размер, первый байт в массиве - это адрес устройства.
void i2c_send_array(uint8_t *Array, uint16_t ArrSize) {
	if (!Ready) {return;} 	// Интерфейс занят.

	SendBufer = Array;		// Сохраняем ссылку на массив
	BuferSize = ArrSize;	// и размер массива.

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
  	static uint16_t BufferPos;		// Текущая позиция в массиве.
  	Status = TWSR & TWSR_MASK;		// Текущий статус TWI.

	switch (Status) {
		case 0x08:							// Передан сигнал START.
			TWDR = (SendBufer[0] << 1) | 0;		// Записываем адрес в регистр.
			TWCR = (1 << TWINT)	|
				(1 << TWEN) |		
				(1 << TWIE);					// Отправляем.
			BufferPos = 1;						// Сброс позиции в массиве.
			break;
		case 0x10:							// Передан сигнал REPEATED START.
			TWDR = (SendBufer[0] << 1) | 0;	// Записываем адрес в регистр.
			TWCR = (1 << TWINT)	|
				(1 << TWEN) |		
				(1 << TWIE);					// Отправляем.
			BufferPos = 1;						// Сброс позиции в массиве.
			break;

	/*** Статусные коды ведущего передатчика ***/

		case 0x18:							// Передан SLA+W, принят ACK.
			TWDR = SendBufer[BufferPos];	// Записываем байт в регистр.
			BufferPos++;
			TWCR = (1 << TWINT)	|
				(1 << TWEN) |		
				(1 << TWIE);					// Отправляем.	
			break;
		case 0x28:								// Передан байт данных, принят ACK.
			if (BufferPos < BuferSize) {
				TWDR = SendBufer[BufferPos];	// Записываем очередной байт в регистр.
				BufferPos++;
				TWCR |= (1 << TWINT);				// Отправляем.	
			}
			else {									// Достигнут конец массива.
				TWCR = (1 << TWINT) |				// Формируем состояние СТОП.			
					(1 << TWEN) |
					(1 << TWSTO);				
				Status = TWI_SUCCESS;				// Передача завершена успешно.
				Ready = 1;							// Интерфейс свободен.
			}
			break;

	/*** Коды ошибок ведущего передатчика ***/

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

	/*** Все прочее пока считаем как ошибку ***/
		default:
			TWCR = (1 << TWINT) |				// Передача не удалась.
					(1 << TWEN) |
					(1 << TWSTO);				// Формируем состояние СТОП.
			Ready = 1;							// Интерфейс свободен. 							
			break;
	}

	// char buffer [12];
	// sprintf(buffer, "0x%x\t0x%x\r\n", Status, TWDR);
	// uart_puts(buffer);
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