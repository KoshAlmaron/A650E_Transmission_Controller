#include <stdint.h>			// Коротние название int.
#include <util/delay.h>		// Задержки.

#include "lcd.h"			// Свой заголовок.
#include "i2c.h"			// I2C (TWI).
#include "configuration.h"	// Настройки.

static uint8_t LCDPort = 0;	// Состояние порта данных дисплея.
static uint8_t SendArray[2];	// Массив для передачи, адрес + состояние порта.

// Буфер строк на экране.
static uint8_t DataBuffer[4][21];

static uint8_t LCDReady = 0;
static uint8_t LastRow = 0;
static uint8_t LastCol = 0;

// Прототипы локальных функций.
static void lcd_port_update();
static void lcd_send_byte(uint8_t Data, uint8_t Com);
static void lcd_set_cursor(uint8_t Row, uint8_t Col);

// Прототипы функций.
void loop_main(uint8_t Wait);		// Прототип функций из main.c.

// Настройка дисплея, необходимо передать адрес и тип.
void lcd_init(uint8_t Addr) {
	SendArray[0] = Addr;

	_delay_ms(150);			// Пауза перед стартом дисплея.
	lcd_send_byte(0x02, 1);	// Установка 4-х битного интерфейса.
	_delay_ms(1);
	lcd_send_byte(0x28, 1);	// 2 строки, симовол 5х7 в 4-х битном режиме.
	lcd_send_byte(0x0c, 1);	// Включение дисплея и отключение курсора.
	lcd_send_byte(0x06, 1);	// Автосдвиг курсора вправо.
	lcd_send_byte(0x01, 1);	// Очистка экрана (1.6 мс). 
	_delay_ms(2);

	DataBuffer[0][0] = 0x0;
	DataBuffer[1][0] = 0x40;
	DataBuffer[2][0] = 0x14;
	DataBuffer[3][0] = 0x54;
}

void lcd_update_buffer(uint8_t Row, char* Array) {
	char* CharIn = 0;
	for (uint8_t i = 0; i < 20; i++) {
		CharIn = &Array[i];
		DataBuffer[Row][i + 1] = *CharIn;
	}
}

void lcd_send_buffer() {
	LCDReady = 0;
	LastRow = 0;
	LastCol = 0;
}

uint8_t lcd_is_ready() {return LCDReady;}

void lcd_process_step() {
	if (LCDReady) {return;}

	if (!LastCol) {
		lcd_set_cursor(LastRow, LastCol);
	}
	else {
		lcd_send_byte(DataBuffer[LastRow][LastCol], 0);
	}

	LastCol++;
	if (LastCol > 20) {
		LastCol = 0;
		LastRow++;

		if (LastRow == 4) {
			LastRow = 0;
			LCDReady = 1;
		}
	}
}

static void lcd_set_cursor(uint8_t Row, uint8_t Col) {
	uint8_t Data = 0;

	switch (Row) {
		case 1:
			Data = 0x40;
			break;
		case 2:
			Data = 0x14;
			break;
		case 3:
			Data = 0x54;
			break;
	}

	Data += Col;		// Добавляем столбец.
	Data |= (1 << 7);	// D7 в единицу.
	lcd_send_byte(Data , 1);
}

static void lcd_port_update() {
	SendArray[1] = LCDPort;
	while (!(i2c_ready()));
	i2c_send_data(SendArray, 2);

	_delay_us(50);						// Задержка на чтение команды.
} 

static void lcd_send_byte(uint8_t Data, uint8_t Com) {
	LCDPort = 0;

	LCDPort |= (1 << 3);				// Подсветка.
	if (!Com) {LCDPort |= (1 << 0);}	// Данные.

	LCDPort |= (Data & 0xF0); 			// Первый полубайт.
	LCDPort |= (1 << 2);				// Е (P2) в единицу.
	lcd_port_update();
	
	LCDPort &= ~(1 << 2);				// Е (P2) в ноль.
	lcd_port_update();

	LCDPort &= 0x0F;
	LCDPort |=  (Data << 4);		 	// Второй полубайт.
	LCDPort |= (1 << 2);				// Е (P2) в единицу.
	lcd_port_update();

	LCDPort &= ~(1 << 2);				// Е (P2) в ноль.
	lcd_port_update();
}

/*
	Выводы LCD	Выводы модуля
	RS	P0
	R/W	P1
	E	P2
	Led	P3
	D4	P4
	D5	P5
	D6	P6
	D7	P7
*/