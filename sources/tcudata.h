// 	Расчет и хранение всех необходимых параметров.

#ifndef _TCUDATA_H_
	#define _TCUDATA_H_

	void calculate_tcu_data();
	int16_t get_oil_temp();
	void calc_tps();

	uint8_t get_slt_pressure();
	int8_t get_slt_temp_corr(uint8_t Value);
	uint8_t get_sln_pressure();
	uint8_t get_slu_pressure_gear2();
	int8_t get_slu_gear2_temp_corr(uint8_t Value);
	uint8_t get_slu_pressure_gear3();
	int16_t rpm_delta(uint8_t Gear);

	uint8_t get_tps_index(uint8_t TPS);
	uint8_t get_temp_index(int16_t Temp);

	void save_gear2_adaptation(int8_t Value);

	extern struct TCU_t TCU; 		// Делаем структуру внешней.

	// Размеры массивов.
	#define TPS_GRID_SIZE 21 
	#define TEMP_GRID_SIZE 37

	// Сетки стандартных осей.
	extern int16_t TempGrid[];
	extern int16_t TPSGrid[];

	extern uint16_t SLTGraph[];
	extern int16_t SLTTempCorrGraph[];
	extern uint16_t SLNGraph[];
	extern uint16_t SLUGear2Graph[];
	extern int16_t SLUGear2TempCorrGraph[];
	extern uint16_t Gear2DeltaRPM[];
	extern int16_t SLUGear3AddGraph[];

	extern int16_t SLUGear2TPSAdaptGraph[];
	extern int16_t SLUGear2TempAdaptGraph[];

	// Структура для хранения переменных.
	typedef struct TCU_t {
		uint16_t DrumRPM;			// Обороты корзины овердрайва.
		uint16_t OutputRPM;			// Обороты выходного вала.
		uint8_t	CarSpeed;			// Скорость автомобиля.
		uint16_t SpdTimerVal;		// Значение регистра сравнения для таймера спидометра.
		int16_t OilTemp;			// Температура масла.
		uint16_t TPS;				// ДПДЗ усредненый для расчетов.
		uint16_t InstTPS;			// ДПДЗ мгновенный.
		uint8_t	SLT;				// ШИМ, линейное давление.
		uint8_t	SLN;				// ШИМ, давление в гидроаккумуляторах.
		uint8_t	SLU;				// ШИМ, давление блокировки гидротрансформатора.
		uint8_t	S1;					// Соленоид № 1.
		uint8_t	S2;					// Соленоид № 2.
		uint8_t	S3;					// Соленоид № 3.
		uint8_t	S4;					// Соленоид № 4.
		uint8_t Selector;			// Положение селектора.
		uint8_t ATMode;				// Режим АКПП.
		int8_t Gear;				// Текущая передача.
		int8_t GearChange;			// Флаг идет процес переключения.
		uint8_t Break;				// Состояние педали тормоза.
		uint8_t EngineWork;			// Флаг работы двигателя.
		uint8_t SlipDetected;		// Обнаружено проскальзывание фрикционов.
		uint8_t Glock;				// Блокировка гидротрансформатора.
		uint8_t	GearUpSpeed;		// Скорость переключения вверх.
		uint8_t	GearDownSpeed;		// Скорость переключения вниз.
	} TCU_t;

#endif