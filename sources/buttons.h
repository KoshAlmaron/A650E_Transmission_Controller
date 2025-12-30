// Кнопки.

#ifndef _BUTTONS_H_
	#define _BUTTONS_H_

	#define TIP_GEAR_UP 	0
	#define TIP_GEAR_DOWN	1

	void buttons_init();
	void buttons_update();
	void buttons_clear();

	uint8_t is_button_press_short(uint8_t N);
	uint8_t is_button_press_long(uint8_t N);

#endif