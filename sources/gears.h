// Фунции переключения передач.

#ifndef _GEARS_H_
	#define _GEARS_H_

	void solenoid_init();

	void gear_control();
	void set_gear_n(int16_t Delay);
	void set_gear_1(int16_t Delay);
	void set_gear_r(int16_t Delay);

	uint8_t get_gear_min_speed();
	uint8_t get_gear_max_speed();

#endif