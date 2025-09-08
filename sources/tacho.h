// Датчики скорости валов.
#ifndef _TACHO_H_
	#define _TACHO_H_

	void tacho_init();
	void tacho_timer();
	uint16_t tacho_get_rpm();

#endif