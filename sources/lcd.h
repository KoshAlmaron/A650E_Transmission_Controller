// Управление LCD на базе HD44780.

#ifndef _LCD_H_
	#define _LCD_H_

	void lcd_init(uint8_t Addr);

	void lcd_update_buffer(uint8_t Row, char* Array);
	void lcd_send_buffer();
	uint8_t lcd_is_ready();
	void lcd_process_step();

#endif