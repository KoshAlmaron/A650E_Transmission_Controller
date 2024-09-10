// Общая логика работы АКПП.

#ifndef _TCULOGIC_H_
	#define _TCULOGIC_H_

	void slt_control();
	void at_mode_control();
	void glock_control(uint8_t Timer);
	void slip_detect();


	void rear_lamp();
	void speedometer_control();
	
#endif

/*
	Так как режим P и N для ЭБУ это одно и тоже,
	то оба положения имеют одинаковое значение (1).
			
			===Режимы АКПП===
			
	0 - Init (Начальное значение при запуске)
	1 - P
	2 - R
	3 - N
	4 - D
	5 - D4
	6 - 3
	7 - L2
	8 - L
	9 - Error
	10 - Manual

*/