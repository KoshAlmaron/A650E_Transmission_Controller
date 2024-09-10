#include <stdint.h>			// Коротние название int.
#include <avr/io.h>			// Названия регистров и номера бит.

#include "gears.h"			// Свой заголовок.
#include "gears_tables.h"	// Таблицы скоростей переключения передач.
#include "pinout.h"			// Список назначенных выводов.
#include "macros.h"			// Макросы.
#include "mathemat.h"		// Математические функции.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "configuration.h"	// Настройки.

int16_t WaitTimer = 0;				// Таймер ожидания из main.
uint16_t GearChangeStep = 100;		// Шаг времени на переключение передачи.
uint16_t AfterChangeDelay = 1000;	// Пауза после вклбчения передачи.

// Максимальная и минимальная передача для каждого режима.
//						  I  P   R  N  D  D4 D3 L2 L  E  M
//						  0  1   2  3  4  5  6  7  8  9  10
const int8_t MaxGear[] = {0, 0, -1, 0, 5, 4, 3, 2, 1, 0, 5};
const int8_t MinGear[] = {0, 0, -1, 0, 1, 1, 1, 2, 1, 0, 1};

uint8_t LastGear2ChangeTPS = 0;	// Значение ДПДЗ при последнем переключении 1>2.
uint8_t LastGear2ChangeSLU = 0;	// Значение SLU при последнем переключении 1>2.

uint8_t LastGear3ChangeTPS = 0;	// Значение ДПДЗ при последнем переключении 2>3.
uint8_t LastGear3ChangeSLU = 0; // Значение SLT при последнем переключении 2>3.

uint8_t LastGear4ChangeTPS = 0;	// Значение ДПДЗ при последнем переключении 3>4.
uint8_t LastGear4ChangeSLT = 0;	// Значение SLT при последнем переключении 3>4.
uint8_t LastGear4ChangeSLN = 0;	// Значение SLN при последнем переключении 3>4.

// Прототипы функций.
void loop_main();	// Прототип функций из main.c.
void glock_control(uint8_t Timer);	// Прототип функций из tculogic.c.

static void gear_change_1_2();
static void gear_change_2_3();
static void gear_change_3_4();
static void gear_change_4_5();

static void gear_change_5_4();
static void gear_change_4_3();
static void gear_change_3_2();
static void gear_change_2_1();

static void gear_up();
static void gear_down();

static void set_gear_change_delays();
static void loop_wait(int16_t Delay);

static uint8_t rpm_after_ok(uint8_t Shift);

//static void set_slt(uint8_t Value);
static void set_sln(uint8_t Value);
static void set_slu(uint8_t Value);

//============================ Начальные передачи =============================
// Включение нейтрали.
void set_gear_n() {
	set_slu(SLU_MIN_VALUE);			// Выключение SLU на случай переключения со второй передачи.
	set_sln(get_sln_pressure()); 	// Заранее устанавливаем давление в гидроаккумуляторах SLN.

	TCU.GearChange = 1;
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	TCU.Gear = 0;
	TCU.GearChange = 0;
	loop_wait(800);				// Пауза после включения нейтрали.
}

// Включение первой передачи.
void set_gear_1() {
	set_slu(SLU_MIN_VALUE);

	TCU.GearChange = 1;

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	TCU.Gear = 1;
	TCU.GearChange = 0;
	loop_wait(800);				// Пауза после включения передачи.
}

// Включение задней передачи.
void set_gear_r() {
	set_slu(SLU_MIN_VALUE);

	TCU.GearChange = 1;
	
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	TCU.Gear = -1;
	TCU.GearChange = 0;
	loop_wait(500);				// Пауза после включения передачи.
}

// Блокировка задней передачи.
void disable_gear_r() {
	set_slu(SLU_MIN_VALUE);
	
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);	// S2 выключает привод.
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
}

//=========================== Переключения вверх ==============================
static void gear_change_1_2() {
	TCU.GearChange = 1;
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_slu(get_slu_pressure_gear2());		// Давление включения и работы второй предачи.

	LastGear2ChangeTPS = TCU.InstTPS;
	LastGear2ChangeSLU = TCU.SLU;

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения УОЗ.
	loop_wait(GearChangeStep * 8);			// Ожидаем срабатывания фрикциона.
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);	// Возврат УОЗ.

	TCU.Gear = 2;
	TCU.GearChange = 0;
}

static void gear_change_2_3() {
	TCU.GearChange = 1;

	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	set_slu(get_slu_pressure_gear3());	// Давление включения и работы второй предачи.
	loop_wait(GearChangeStep * 800);		// Ждем повышения давления в гидроаккумуляторе.

	LastGear3ChangeTPS = TCU.InstTPS;
	LastGear3ChangeSLU = TCU.SLU;

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима 3. 
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения УОЗ.
	loop_wait(GearChangeStep * 10);			// Ожидаем срабатывания фрикциона.
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);	// Возврат УОЗ.
	set_slu(SLU_MIN_VALUE);					// Отключаем давление на B3.

	TCU.Gear = 3;
	TCU.GearChange = 0;
}

static void gear_change_3_4() {
	TCU.GearChange = 1;
	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	loop_wait(GearChangeStep * 10);		// Ждем повышения давления в гидроаккумуляторе.

	LastGear4ChangeTPS = TCU.InstTPS;
	LastGear4ChangeSLT = TCU.SLT;
	LastGear4ChangeSLN = TCU.SLN;

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения УОЗ.
	loop_wait(GearChangeStep * 8);			// Ожидаем срабатывания фрикциона.
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);	// Возврат УОЗ.

	TCU.Gear = 4;
	TCU.GearChange = 0;	
}

static void gear_change_4_5() {
	TCU.GearChange = 1;
	glock_control(100);				// Снижаем давление блокировки гидротрансформатора.
	set_sln(get_sln_pressure()); 	// Соленоид SLN.
	loop_wait(GearChangeStep * 10);	// Ждем повышения давления в гидроаккумуляторе.
	glock_control(100);				// Выключаем блокировку гидротрансформатора.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_HIGH(SOLENOID_S4_PIN);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения УОЗ.
	loop_wait(GearChangeStep * 8);			// Ожидаем срабатывания фрикциона.
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);	// Возврат УОЗ.

	TCU.Gear = 5;
	TCU.GearChange = 0;	
}

//=========================== Переключения вниз ===============================
static void gear_change_5_4() {
	TCU.GearChange = 1;
	glock_control(100);	// Снижаем давление блокировки гидротрансформатора.
	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	loop_wait(GearChangeStep * 10);	// Ждем повышения давления в гидроаккумуляторе.
	glock_control(100);	// Выключаем блокировку гидротрансформатора.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	TCU.Gear = 4;
	TCU.GearChange = 0;		
}

static void gear_change_4_3() {
	TCU.GearChange = 1;
	glock_control(100);	// Снижаем давление блокировки гидротрансформатора.
	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	loop_wait(GearChangeStep * 10);	// Ждем повышения давления в гидроаккумуляторе.
	glock_control(100);	// Выключаем блокировку гидротрансформатора.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима 3. 
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	TCU.Gear = 3;
	TCU.GearChange = 0;	
}

static void gear_change_3_2() {
	TCU.GearChange = 1;

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_slu(get_slu_pressure_gear2());
	loop_wait(GearChangeStep * 12);		// Ждем снижение давления в тормозе B2.

	SET_PIN_LOW(SOLENOID_S3_PIN);		// Отключаем систему "Clutch to Clutch".

	TCU.Gear = 2;
	TCU.GearChange = 0;	
}

static void gear_change_2_1() {
	TCU.GearChange = 1;
	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	loop_wait(GearChangeStep * 10);		// Ждем повышения давления в гидроаккумуляторе.
	
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}
	set_slu(SLU_MIN_VALUE);

	TCU.Gear = 1;
	TCU.GearChange = 0;
}

//========================== Вспомогательные функции ==========================

// Настройка выходов шифтовых селеноидов, а также лампы заднего хода.
void solenoid_init() {
	SET_PIN_MODE_OUTPUT(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_MODE_OUTPUT(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_MODE_OUTPUT(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_MODE_OUTPUT(SOLENOID_S4_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	SET_PIN_MODE_OUTPUT(REAR_LAMP_PIN);
	SET_PIN_LOW(REAR_LAMP_PIN);

	// Запрос снижения мощности.
	SET_PIN_MODE_OUTPUT(REQUEST_POWER_DOWN_PIN);
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);	
}

// Обновление порогов переключения передач.
void update_gear_speed() {
	TCU.GearUpSpeed = get_gear_max_speed(TCU.Gear);		// Верхняя граница переключения.
	TCU.GearDownSpeed = get_gear_min_speed(TCU.Gear);	// Нижняя граница переключения.
}

// Контроль переключения передач, возвращает время задержки после переключения.
uint16_t gear_control() {
	// Только режимы D - L.
	if (TCU.ATMode < 4 || TCU.ATMode > 8) {return 0;}
	if (TCU.Gear < 1 || TCU.Gear > 5) {return 0;}

	set_gear_change_delays();	// Установка времени на переключение от ДПДЗ.

	TCU.GearUpSpeed = get_gear_max_speed(TCU.Gear);		// Верхняя граница переключения.
	TCU.GearDownSpeed = get_gear_min_speed(TCU.Gear);	// Нижняя граница переключения.

	// Переключения при изменение режима АКПП.
	if (TCU.Gear > MaxGear[TCU.ATMode]) {	
		// Проверка оборотов просле переключения.
		if (rpm_after_ok(-1)) {gear_down();}
		return AfterChangeDelay;
	} 
	if (TCU.Gear < MinGear[TCU.ATMode]) {
		// Проверка оборотов просле переключения.
		if (rpm_after_ok(1)) {gear_up();}
		return AfterChangeDelay;
	}

	// Скорость выше порога.
	if (TCU.CarSpeed > TCU.GearUpSpeed) {
		if (TCU.InstTPS > 2) {					// Не повышать передачу при сбросе газа.
			if (rpm_after_ok(1)) {gear_up();}
		}
		return AfterChangeDelay;
	}

	// Скорость ниже порога.
	if (TCU.CarSpeed < TCU.GearDownSpeed) {
		if (rpm_after_ok(-1)) {gear_down();}
		return AfterChangeDelay;
	}
	return 0;
}

// Управление давлением SLU B3 для работы второй передачи.
void slu_gear2_control() {
	if (TCU.Gear != 2) {return;}	// Управление давлением SLU для второй передачи.
	set_slu(get_slu_pressure_gear2());
}

// Переключение вверх.
static void gear_up() {
	if (TCU.Gear >= MaxGear[TCU.ATMode]) {return;}

	switch (TCU.Gear) {
		case 1:
			gear_change_1_2();
			break;
		case 2:
			gear_change_2_3();
			break;
		case 3:
			gear_change_3_4();
			break;
		case 4:
			gear_change_4_5();
			break;
	}
}

// Переключение вниз.
static void gear_down() {
	if (TCU.Gear <= MinGear[TCU.ATMode]) {return;}

	switch (TCU.Gear) {
		case 2:
			gear_change_2_1();
			break;
		case 3:
			// При ХХ сразу переходить на первую передачу.
			//if (TCU.InstTPS < TPS_IDLE_LIMIT) {gear_change_3_1();}
			//else {gear_change_3_2();}
			gear_change_3_2();
			break;
		case 4:
			gear_change_4_3();
			break;
		case 5:
			gear_change_5_4();
			break;
	}	
}

static void set_gear_change_delays() {
	GearChangeStep = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, GearChangeStepArray, TPS_GRID_SIZE);
	AfterChangeDelay = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, AfterChangeDelayArray, TPS_GRID_SIZE);
}

// Ожидание с основным циклом.
static void loop_wait(int16_t Delay) {
	WaitTimer = -1 * Delay;		// Устанавливаем время ожидания.
	while (WaitTimer < 0) {
		loop_main();
	}
}

uint8_t get_gear_max_speed(int8_t Gear) {
	if (Gear == 5 || Gear <= 0) {return 130;}

	uint16_t* Array;
	switch (Gear) {
		case 1:
			Array = Gear_1_2;
			break;
		case 2:
			Array = Gear_2_3;
			break;
		case 3:
			Array = Gear_3_4;
			break;
		case 4:
			Array = Gear_4_5;
			break;
		default:
			return 0;
	}
	return get_interpolated_value_uint16_t(TCU.TPS, TPSGrid, Array, TPS_GRID_SIZE);
}

uint8_t get_gear_min_speed(int8_t Gear) {
	if (Gear <= 1) {return 5;}

	uint16_t* Array;
	switch (Gear) {
		case 2:
			Array = Gear_2_1;
			break;
		case 3:
			Array = Gear_3_2;
			break;
		case 4:
			Array = Gear_4_3;
			break;
		case 5:
			Array = Gear_5_4;
			break;
		default:
			return 0;
	}
	return get_interpolated_value_uint16_t(TCU.TPS, TPSGrid, Array, TPS_GRID_SIZE);
}

static uint8_t rpm_after_ok(uint8_t Shift) {
	// Разрешить переключение стоя на тормозе на ХХ.
	if (!TCU.OutputRPM && TCU.InstTPS < TPS_IDLE_LIMIT && TCU.Break) {return 1;}

	uint8_t Gear = TCU.Gear + Shift;
	uint16_t NewRPM = 0;
	switch (Gear) {
		case 1:
			NewRPM = ((uint32_t) TCU.OutputRPM * GEAR_1_RATIO) >> 10;
			break;
		case 2:
			NewRPM = ((uint32_t) TCU.OutputRPM * GEAR_2_RATIO) >> 10;
			break;
		case 3:
			NewRPM = ((uint32_t) TCU.OutputRPM * GEAR_3_RATIO) >> 10;
			break;
		case 4:
			NewRPM = ((uint32_t) TCU.OutputRPM * GEAR_4_RATIO) >> 10;
			break;
		case 5:
			NewRPM = ((uint32_t) TCU.OutputRPM * GEAR_5_RATIO) >> 10;
			break;
	}

	if (NewRPM > RPM_MIN && NewRPM < RPM_MAX) {return 1;}
	else {return 0;}
}

// static void set_slt(uint8_t Value) {
// 	TCU.SLT = Value;
// 	OCR1A = TCU.SLT;	// SLT - выход A таймера 1.	
// }

static void set_sln(uint8_t Value) {
	TCU.SLN = Value;
	OCR1B = TCU.SLN;	// SLN - выход B таймера 1.	
}

static void set_slu(uint8_t Value) {
	TCU.SLU = Value;
	OCR1C = TCU.SLU;	// SLU - выход C таймера 1.	
}
