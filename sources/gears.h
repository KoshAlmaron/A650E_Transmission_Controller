// Фунции переключения передач.

#ifndef _GEARS_H_
	#define _GEARS_H_

	void set_gear_n();
	void set_gear_1();
	void set_gear_r();
	void disable_gear_r();

	void solenoid_init();
	void update_gear_speed();
	uint16_t gear_control();
	void slu_b3_control();

#endif