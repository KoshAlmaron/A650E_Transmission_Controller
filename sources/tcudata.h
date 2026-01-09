// 	Расчет и хранение всех необходимых параметров.

#ifndef _TCUDATA_H_
	#define _TCUDATA_H_

	void calculate_tcu_data();
	uint16_t get_speed_timer_value();
	int16_t get_oil_temp();
	void calc_tps();

	uint16_t get_slt_pressure();
	int16_t get_slt_temp_corr(int16_t Value);
	uint16_t get_sln_pressure();
	int16_t get_sln_temp_corr(int16_t Value);
	
	uint16_t get_slu_pressure_gear2();
	int16_t get_slu_gear2_temp_corr(int16_t Value);
	int16_t get_gear2_rpm_adv();
	
	uint16_t get_slu_pressure_gear3();
	uint16_t get_gear3_slu_delay();
	uint16_t get_sln_pressure_gear3();
	int16_t get_gear3_sln_offset();
	
	int16_t rpm_delta(uint8_t Gear);

	uint8_t get_tps_index(uint8_t TPS);
	uint8_t get_temp_index(int16_t Temp);
	uint8_t get_delta_rpm_index(int16_t DeltaRPM);

	void save_gear2_slu_adaptation(int8_t Value, uint8_t TPS);
	void save_gear2_adv_adaptation(int8_t Value, int16_t InitDrumRPMDelta);
	void save_gear3_slu_adaptation(int8_t Value, uint8_t TPS);

	// Структура для хранения переменных.
	typedef struct TCU_t {
		uint16_t EngineRPM;			// Обороты двигателя.
		uint16_t DrumRPM;			// Обороты корзины овердрайва.
		int16_t DrumRPMDelta;		// Скорость изменения оборотов корзины овердрайва.		
		uint16_t OutputRPM;			// Обороты выходного вала.
		uint8_t CarSpeed;			// Скорость автомобиля.
		int16_t OilTemp;			// Температура масла.
		uint16_t TPS;				// ДПДЗ усредненый для расчетов.
		uint16_t InstTPS;			// ДПДЗ мгновенный.
		uint16_t Load;				// Такущая нагрузка (ДПДЗ + барокоррекция).
		uint16_t Barometer;			// Атмосферное давление (x0.1 кПа).
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
		uint16_t RawTPS;			// Сырые значения АЦП ДПДЗ.
		uint16_t RawOIL;			// Сырые значения АЦП температуры масла.
		int8_t AdaptationFlagTPS;	// Флаг срабатывания адаптации по ДПДЗ.
		int8_t AdaptationFlagTemp;	// Флаг срабатывания адаптации по температуре.
		uint16_t ManualModeTimer;	// Режим ручного переключения передач (Типтроник).
	} TCU_t;

	extern struct TCU_t TCU; 	// Делаем структуру с параметрами внешней.

	// Размеры массивов.
	#define TPS_GRID_SIZE 21 
	#define TEMP_GRID_SIZE 31
	#define DELTA_RPM_GRID_SIZE 21

	//================================ Сетки осей =============================
	typedef struct GRIDS_t {
		int16_t TPSGrid[TPS_GRID_SIZE];				// Сетка оси ДПДЗ.
		int16_t TempGrid[TEMP_GRID_SIZE];			// Сетка оси температуры. 
		int16_t DeltaRPMGrid[DELTA_RPM_GRID_SIZE];	// Сетка оси ускоренияы.
	} GRIDS_t;
	extern struct GRIDS_t GRIDS; 	// Сетки стандартных осей.

	//=================================== Датчики =============================
	typedef struct ADCTBL_t {
		int16_t TPSGraph[TPS_GRID_SIZE];		// ДПДЗ (показания АЦП).
		int16_t OilTempGraph[TEMP_GRID_SIZE];	// Температура масла (показания АЦП).
	} ADCTBL_t;
	extern struct ADCTBL_t ADCTBL; // Таблицы АЦП.

	//================================== Адаптация ============================
	typedef struct ADAPT_t {
		int16_t SLUGear2TPSAdaptGraph[TPS_GRID_SIZE];	// Адаптация по ДПДЗ включения второй передачи.
		int16_t SLUGear2TempAdaptGraph[TEMP_GRID_SIZE];	// Адаптация по температуре включения второй передачи.

		int16_t Gear2AdvAdaptGraph[TPS_GRID_SIZE];		// Адаптация оборотов реактивации второй передачи.
		int16_t Gear2AdvTempAdaptGraph[TEMP_GRID_SIZE];	// Адаптация оборотов по температуре.

		int16_t SLUGear3TPSAdaptGraph[TPS_GRID_SIZE];	// Адаптация по ДПДЗ включения третьей передачи.
		int16_t SLUGear3TempAdaptGraph[TEMP_GRID_SIZE];	// Адаптация по температуре включения третьей передачи.
	} ADAPT_t;
	extern struct ADAPT_t ADAPT; // Таблицы адаптаций.

	//============================== Основные таблицы =========================
	typedef struct TABLES_t {
		uint16_t SLTGraph[TPS_GRID_SIZE];			// Линейное давление SLT от ДПДЗ.
		int16_t SLTTempCorrGraph[TEMP_GRID_SIZE];	// Коррекция в % давления SLT от температуры.
		
		uint16_t SLNGraph[TPS_GRID_SIZE];			// Давление SLN от ДПДЗ (Величина сброса давления).
		int16_t SLNTempCorrGraph[TEMP_GRID_SIZE];	// Коррекция в % давления SLN от температуры.

		uint16_t SLUGear2Graph[TPS_GRID_SIZE];			// Давление SLU включения второй передачи (SLU B3) от ДПДЗ.
		int16_t SLUGear2TempCorrGraph[TEMP_GRID_SIZE];	// Корекция в % давления в тормозе второй передачи B3 от температуры.
		int16_t Gear2AdvGraph[DELTA_RPM_GRID_SIZE];		// Опережение по оборотам реактивации второй передачи.
		int16_t Gear2AdvTempCorrGraph[TEMP_GRID_SIZE];	// Коррекция по температуре.

		uint16_t SLUGear3Graph[TPS_GRID_SIZE];				// Давление SLU включения третьей передачи (SLU B2) от ДПДЗ.
		uint16_t SLUGear3DelayGraph[TPS_GRID_SIZE];			// Время удержания SLU от ДПДЗ при включении третьей передачи.
		int16_t SLUG3DelayTempCorrGraph[TEMP_GRID_SIZE];	// Корекция времени удержания SLU при включении третьей передачи от температуры (мс).
		uint16_t SLNGear3Graph[TPS_GRID_SIZE];				// Давление SLN при включении третьей передачи.
		int16_t SLNGear3OffsetGraph[TPS_GRID_SIZE];			// Смещение времени включения SLN при включении третьей передачи.

		uint16_t GearChangeStepArray[TPS_GRID_SIZE];	// Длина одного шага времени на переключение от ДПДЗ.
	} TABLES_t;
	extern struct TABLES_t TABLES; // Основные таблицы.

	//================ Скорости для переключения передач от ДПДЗ ==============
	typedef struct SPEED_t {
		uint8_t Gear_2_1[TPS_GRID_SIZE];
		uint8_t Gear_1_2[TPS_GRID_SIZE];

		uint8_t Gear_3_2[TPS_GRID_SIZE];
		uint8_t Gear_2_3[TPS_GRID_SIZE];

		uint8_t Gear_4_3[TPS_GRID_SIZE];
		uint8_t Gear_3_4[TPS_GRID_SIZE];

		uint8_t Gear_5_4[TPS_GRID_SIZE];
		uint8_t Gear_4_5[TPS_GRID_SIZE];

	} SPEED_t;
	extern struct SPEED_t SPEED; // Скорости для переключения передач.

	extern uint8_t SpeedTestFlag;	// Флаг включения тестирования скорости.

	#define SLT_GRAPH							0
	#define SLT_TEMP_CORR_GRAPH					1
	#define SLN_GRAPH							2
	#define SLN_TEMP_CORR_GRAPH					3
	#define SLU_GEAR2_GRAPH						4
	#define SLU_GEAR2_TPS_ADAPT_GRAPH			5
	#define SLU_GEAR2_TEMP_CORR_GRAPH			6
	#define SLU_GEAR2_TEMP_ADAPT_GRAPH			7
	#define GEAR_CHANGE_STEP_ARRAY				8
	#define GEAR2_ADV_GRAPH						9
	#define GEAR2_ADV_ADAPT_GRAPH				10
	#define GEAR2_ADV_TEMP_CORR_GRAPH			11
	#define GEAR2_ADV_TEMP_ADAPT_GRAPH			12
	#define SLU_GEAR3_GRAPH						13
	#define SLU_GEAR3_DELAY_GRAPH				14
	#define SLU_GEAR3_TPS_ADAPT_GRAPH			15
	#define SLU_G3_DELAY_TEMP_CORR_GRAPH		16
	#define SLU_GEAR3_TEMP_ADAPT_GRAPH			17
	#define SLN_GEAR3_GRAPH						18
	#define SLN_GEAR3_OFFSET_GRAPH				19
	#define TPS_ADC_GRAPH						20
	#define OIL_ADC_GRAPH						21
	#define GEAR_SPEED_GRAPHS					22

#endif