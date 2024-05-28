// Датчики скорости валов.
#ifndef _SPEED_H_
	#define _SPEED_H_

	void calculate_overdrive_drum_rpm();
	void calculate_output_shaft_rpm();
	void speed_sensors_read();
	uint16_t get_overdrive_drum_rpm();
	uint16_t get_output_shaft_rpm();
	
#endif