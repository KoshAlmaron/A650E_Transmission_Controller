#include <stdint.h>			// Коротние название int.
#include <util/delay.h>		// Задержки.
#include "lcd.h"			// Свой заголовок.
#include "i2c.h"			// I2C (TWI).

uint8_t LCDPort = 0;	// Состояние порта данных дисплея.
uint8_t SendArray[2];	// Массив для передачи, адрес + состояние порта.

// Прототипы локальных функций.
static void lcd_port_update();
static void lcd_send_byte(uint8_t Data, uint8_t Com);

// Настройка дисплея, необходимо передать адрес и тип.
void lcd_init(uint8_t Addr) {
	SendArray[0] = Addr;

	_delay_ms(200);			// Пауза перед стартом дисплея.
	lcd_send_byte(0x02, 1);	// Установка 4-х битного интерфейса.
	_delay_ms(1);
	lcd_send_byte(0x28, 1);	// 2 строки, симовол 5х7 в 4-х битном режиме.
	lcd_send_byte(0x0c, 1);	// Включение дисплея и отключение курсора.
	lcd_send_byte(0x06, 1);	// Автосдвиг курсора вправо.
	lcd_send_byte(0x01, 1);	// Очистка экрана (1.6 мс). 
	_delay_ms(2);
}

void lcd_set_cursor(uint8_t Row, uint8_t Col) {
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
	_delay_ms(2);
}

static void lcd_port_update() {
	SendArray[1] = LCDPort;
	while (!(i2c_ready()));
	i2c_send_array(SendArray, 2);
} 

static void lcd_send_byte(uint8_t Data, uint8_t Com) {
	LCDPort = 0;

	LCDPort |= (1 << 3);				// Подсветка.
	if (!Com) {LCDPort |= (1 << 0);}	// Данные.

	LCDPort |= (Data & 0xF0); 			// Первый полубайт.
	LCDPort |= (1 << 2);				// Е (P2) в единицу.
	
	lcd_port_update();
	_delay_us(150);						// Задержка на чтение команды.
	LCDPort &= ~(1 << 2);				// Е (P2) в ноль.
	lcd_port_update();
	_delay_us(150);						// Задержка на выполнение команды.
	
	LCDPort &= 0x0F;
	LCDPort |=  (Data << 4);		 	// Второй полубайт.
	LCDPort |= (1 << 2);				// Е (P2) в единицу.
	
	lcd_port_update();
	_delay_us(150);						// Задержка на чтение команды.
	LCDPort &= ~(1 << 2);				// Е (P2) в ноль.
	lcd_port_update();
	_delay_us(150);						// Задержка на выполнение команды.
}

void lcd_send_char(char Char) {
	// Чтобы компилятор не ругался, преобразуем char в uint_8t.
	char* CharIn = &Char;
	uint8_t ByteOut = *CharIn;
	lcd_send_byte(ByteOut, 0);	
}

void lcd_send_string(char* Array, uint8_t MsgSize) {
	// Чтобы компилятор не ругался, преобразуем char в uint_8t.
	for (uint8_t i = 0; i < MsgSize;  i++) {
		char* CharIn = &Array[i];
		uint8_t ByteOut = *CharIn;
		lcd_send_byte(ByteOut, 0);
	}
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