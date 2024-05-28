// Управление LCD на базе HD44780.

#ifndef _LCD_H_
	#define _LCD_H_

	void lcd_init(uint8_t Addr);
	void lcd_set_cursor(uint8_t Row, uint8_t Col);
	void lcd_send_char(char Char);
	void lcd_send_string(char* Array, uint8_t Size);

#endif