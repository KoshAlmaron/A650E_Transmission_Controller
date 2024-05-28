// Настройки.

#ifndef _CONFIGURATION_H_
	#define _CONFIGURATION_H_

	// Количество зубов на валах.
	#define OVERDRIVE_DRUM_TEETH_COUNT 16
	#define OUTPUT_SHAFT_TEETH_COUNT 12

	// Колитчество импульсов на 1 км для спидометра.
	#define SPEED_INPULS_PER_KM 6000

	// Добавка давления SLT в режиме "R" и "1". +10% 255 * 0.1 = 26.
	#define SLT_ADD_R1 26

	// Лимиты оборотов после переключения.
	#define RPM_MIN	1400
	#define RPM_MAX	4800

	// Передаточные числа x1024.
	#define GEAR_1_RATIO 3438
	#define GEAR_2_RATIO 2232
	#define GEAR_3_RATIO 1458
	#define GEAR_4_RATIO 1024
	#define GEAR_5_RATIO 771

#endif
/*
	Датчик на входном валу  замеряет скорость сцепления 
	овердрайва, а не первичного вала.
	Поэтому на пятой передаче его обороты не соответствуют
	оборотам первичного вала.
*/
