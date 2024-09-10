// Фунции переключения передач.

#ifndef _GEARS_H_
	#define _GEARS_H_

	void set_gear_n();
	void set_gear_1();
	void set_gear_r();
	void disable_gear_r();

	void solenoid_init();
	void update_gear_speed();
	void gear_control();
	uint8_t get_gear_min_speed();
	uint8_t get_gear_max_speed();

#endif