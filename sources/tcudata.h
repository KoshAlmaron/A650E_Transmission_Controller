// 	Расчет и хранение всех необходимых параметров.

#ifndef _TCUDATA_H_
	#define _TCUDATA_H_

	void calculate_tcu_data();
	int16_t get_oil_temp();
	void calc_tps();

	uint16_t get_slt_pressure();
	int16_t get_slt_temp_corr(int16_t Value);
	uint16_t get_sln_pressure();
	
	uint16_t get_slu_pressure_gear2();
	int16_t get_slu_gear2_temp_corr(int16_t Value);
	uint16_t get_slu_add_gear2();
	
	uint16_t get_slu_pressure_gear3();
	uint16_t get_gear3_slu_delay(uint8_t TPS);
	int16_t get_gear3_sln_offset(uint8_t TPS);
	
	int16_t rpm_delta(uint8_t Gear);

	uint8_t get_tps_index(uint8_t TPS);
	uint8_t get_temp_index(int16_t Temp);

	void save_gear2_adaptation(int8_t Value);

	// Структура для хранения переменных.
	typedef struct TCU_t {
		uint16_t DrumRPM;			// Обороты корзины овердрайва.
		uint16_t OutputRPM;			// Обороты выходного вала.
		uint8_t CarSpeed;			// Скорость автомобиля.
		int16_t OilTemp;			// Температура масла.
		uint16_t TPS;				// ДПДЗ усредненый для расчетов.
		uint16_t InstTPS;			// ДПДЗ мгновенный.
		uint16_t SLT;				// ШИМ, линейное давление.
		uint16_t SLN;				// ШИМ, давление в гидроаккумуляторах.
		uint16_t SLU;				// ШИМ, давление блокировки гидротрансформатора.
		uint8_t S1;					// Соленоид № 1.
		uint8_t S2;					// Соленоид № 2.
		uint8_t S3;					// Соленоид № 3.
		uint8_t S4;					// Соленоид № 4.
		uint8_t Selector;			// Положение селектора.
		uint8_t ATMode;				// Режим АКПП.
		int8_t Gear;				// Текущая передача.
		int8_t GearChange;			// Флаг идет процес переключения.
		uint8_t GearStep;			// Номер шага процесса переключения.
		uint8_t LastStep;			// Последний шаг при переключении 1>2 или 2>3.
		uint8_t Gear2State;			// Состояние второй передачи.
		uint8_t Break;				// Состояние педали тормоза.
		uint8_t EngineWork;			// Флаг работы двигателя.
		uint8_t SlipDetected;		// Обнаружено проскальзывание фрикционов.
		uint8_t Glock;				// Блокировка гидротрансформатора.
		uint8_t GearUpSpeed;		// Скорость переключения вверх.
		uint8_t GearDownSpeed;		// Скорость переключения вниз.
		uint8_t GearChangeTPS;		// ДПДЗ в начале переключения.
		uint16_t GearChangeSLT;		// SLT в начале переключения.
		uint16_t GearChangeSLN;		// SLN в начале переключения.
		uint16_t GearChangeSLU;		// SLU в начале переключения.
		uint16_t LastPDRTime;		// Последнее время работы PDR.
		uint16_t CycleTime;			// Время цикла.
		uint8_t DebugMode;			// Режим отладки.
	} TCU_t;

	extern struct TCU_t TCU; 		// Делаем структуру внешней.

	// Размеры массивов.
	#define TPS_GRID_SIZE 21 
	#define TEMP_GRID_SIZE 31

	// Сетки стандартных осей.
	extern int16_t TempGrid[];
	extern int16_t TPSGrid[];

	extern uint16_t SLTGraph[];
	extern int16_t SLTTempCorrGraph[];

	extern uint16_t SLNGraph[];

	extern uint16_t SLUGear2Graph[];
	extern int16_t SLUGear2TempCorrGraph[];
	extern int16_t SLUGear2AddGraph[];

	extern int16_t SLUGear2TPSAdaptGraph[];
	extern int16_t SLUGear2TempAdaptGraph[];

	extern uint16_t SLUGear3Graph[];
	extern uint16_t SLUGear3DelayGraph[];
	extern int16_t SLNGear3OffsetGraph[];

#endif