//Нештатный код/patch
// Датчики скорости валов.
#ifndef _TACHO_H_
	#define _TACHO_H_

	void tacho_init();
	void tacho_timer(uint8_t TimerAdd);
	uint8_t get_tacho_ew();
#endif