// Настройки.

#ifndef _CONFIGURATION_H_
	#define _CONFIGURATION_H_

	// Вывод отладочной информации в UART.
	//#define DEBUG_MODE_PRINT

	// Лимиты оборотов после переключения.
	#define RPM_MIN	1000
	#define RPM_MAX	4700

	#define TPS_IDLE_LIMIT 3			// Порог значения ДПДЗ для ХХ.
	#define MAX_SLIP_RPM 60				// Разница в оборотах валов для обнаружения проскальзывания.

	#define SLN_MIN_PRESSURE 500		// Значение ШИМ для минимального давления подпора гидроаккумуляторов.
	#define SLN_IDLE_PRESSURE 72		// Значение ШИМ для максимального давления подпора гидроаккумуляторов.

	#define SLU_MIN_VALUE 72			// Постоянное минимальное давление SLU.
	#define SLU_GLOCK_START_VALUE 292	// Давление схватвания блокировки гидротрансформатора.
	#define SLU_GLOCK_MAX_VALUE 700		// Максимальное давление блокировки гидротрансформатора.

	#define GLOCK_MAX_TPS 26			// Максимальное положение дросселя для блокировки гидротрансформатора.

	#define PDR_MAX_TPS 33						// Ограничение активации PDR по дросселю.
	#define GEAR_2_SLU_ADAPTATION_MAX_TPS 50	// Ограничение адаптации по ДПДЗ.
	
	// Расскомментировать для активации.
	#define GEAR_2_SLU_TPS_ADAPTATION		// Авто коррекция SLU по ДПДЗ при прогретом масле.
	#define GEAR_2_SLU_TEMP_ADAPTATION		// Авто коррекция SLU по температуре.

	// Инверсия ШИМ соленоидов для разных вариантов железа.
	//#define INVERSE_SLT_PWM
	//#define INVERSE_SLN_PWM
	//#define INVERSE_SLU_PWM

//=============================================================================
//===================== Неизменяемые параметры АКПП ===========================
//=============================================================================

	// Передаточные числа x1024.
	#define GEAR_1_RATIO 3438
	#define GEAR_2_RATIO 2232
	#define GEAR_3_RATIO 1458
	#define GEAR_4_RATIO 1024
	#define GEAR_5_RATIO 771

	// Количество зубов на валах.
	#define OVERDRIVE_DRUM_TEETH_COUNT 16
	#define OUTPUT_SHAFT_TEETH_COUNT 12

	// Количество импульсов на 1 км для спидометра.
	#define SPEED_INPULS_PER_KM 6000

#endif
/*
	Датчик на входном валу  замеряет скорость корзины
	овердрайва, а не первичного вала.
	Поэтому на пятой передаче его обороты не соответствуют
	оборотам первичного вала.
*/
