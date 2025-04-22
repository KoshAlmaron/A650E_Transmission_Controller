// Фунции переключения передач.

#ifndef _GEARS_H_
	#define _GEARS_H_

	void set_gear_n();
	void set_gear_1();
	void set_gear_r();
	void disable_gear_r();

	uint8_t get_gear_min_speed(int8_t Gear);
	uint8_t get_gear_max_speed(int8_t Gear);

	void solenoid_init();
	void update_gear_speed();
	uint16_t gear_control();
	void slu_gear2_control(uint8_t Time);
	void idle_control(uint8_t Time);

#endif