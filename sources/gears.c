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

uint8_t LastGearChangeTPS = 0;	// Значение ДПДЗ при последнем переключении 1>2 или 2>3.
uint8_t LastGearChangeSLU = 0;	// Значение SLU при последнем переключении 1>2 или 2>3.

// Прототипы функций.
void loop_main();	// Прототип функций из main.c.

static void gear_change_1_2();
static void gear_change_2_3();
static void gear_change_3_4();
static void gear_change_4_5();

static void gear_change_5_4();
static void gear_change_4_3();
static void gear_change_3_2();
//static void gear_change_3_1();	// Костыль, так как переключение 3>2 на ХХ вызывает рывки.
static void gear_change_2_1();

static void gear_up();
static void gear_down();

static void set_gear_change_delays();
static void loop_wait(int16_t Delay);

static uint8_t get_gear_min_speed();
static uint8_t get_gear_max_speed();

static uint8_t rpm_after_ok(uint8_t Shift);
//static uint8_t gear_check(uint8_t Gear);

static void set_slt(uint8_t Value);
static void set_sln(uint8_t Value);
static void set_slu(uint8_t Value);

//============================ Начальные передачи =============================
// Включение нейтрали.
void set_gear_n() {
	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.
	set_sln(get_sln_pressure()); 	// Заранее устанавливаем давление в гидроаккумуляторах SLN.

	TCU.GearChange = 1;
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	TCU.Gear = 0;
	TCU.GearChange = 0;
	loop_wait(600);				// Пауза после включения нейтрали.
}

// Включение первой передачи.
void set_gear_1() {
	set_slt(SLT_START_VALUE);
	set_sln(get_sln_pressure());
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
	loop_wait(500);				// Пауза после включения передачи.
}

// Включение задней передачи.
void set_gear_r() {
	set_slt(SLT_START_VALUE);
	set_sln(get_sln_pressure());
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

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.
	
	set_slu(get_slu_pressure_b3());			// Давление включения предачи.

	LastGearChangeTPS = TCU.InstTPS;
	LastGearChangeSLU = TCU.SLU;

	loop_wait(GearChangeStep * 12);			// Ждем повышения давления в тормозе B3.

	TCU.Gear = 2;
	TCU.GearChange = 0;
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);
}

static void gear_change_2_3() {
	TCU.GearChange = 1;

	set_slt(MIN(255, TCU.SLT + SLTB2Add));	// Добавка давления SLT.
	set_sln(get_sln_pressure()); 			// Соленоид SLN.
	set_slu(get_slu_pressure_b2());			// Давление включения предачи.

	loop_wait(GearChangeStep * 10);			// Ждем повышения давления в гидроаккумуляторе.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".
	SET_PIN_LOW(SOLENOID_S4_PIN);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.

	LastGearChangeTPS = TCU.InstTPS;
	LastGearChangeSLU = TCU.SLU;

	loop_wait(GearChangeStep * 12);			// Ожидаем срабатывания фрикциона.

	// Отличие для режима 3. 
	if (TCU.ATMode != 6) {SET_PIN_LOW(SOLENOID_S3_PIN);}
	set_slu(SLU_MIN_VALUE);				// Отключаем давление на B3.

	TCU.Gear = 3;
	TCU.GearChange = 0;
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);
}

static void gear_change_3_4() {
	TCU.GearChange = 1;
	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	loop_wait(GearChangeStep * 6);	// Ждем повышения давления в гидроаккумуляторе.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.

	loop_wait(GearChangeStep * 6);		// Ожидаем срабатывания фрикциона.

	TCU.Gear = 4;
	TCU.GearChange = 0;	
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);
}

static void gear_change_4_5() {
	TCU.GearChange = 1;
	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	loop_wait(GearChangeStep * 6);	// Ждем повышения давления в гидроаккумуляторе.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_HIGH(SOLENOID_S4_PIN);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.

	loop_wait(GearChangeStep * 6);			// Ожидаем срабатывания фрикциона.

	TCU.Gear = 5;
	TCU.GearChange = 0;	
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);
}

//=========================== Переключения вниз ===============================
static void gear_change_5_4() {

	TCU.GearChange = 1;
	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	loop_wait(GearChangeStep * 6);	// Ждем повышения давления в гидроаккумуляторе.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	loop_wait(GearChangeStep * 6);			// Ожидаем срабатывания фрикциона.

	TCU.Gear = 4;
	TCU.GearChange = 0;		
}

static void gear_change_4_3() {
	TCU.GearChange = 1;
	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	loop_wait(GearChangeStep * 6);	// Ждем повышения давления в гидроаккумуляторе.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима 3. 
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	loop_wait(GearChangeStep * 6);			// Ожидаем срабатывания фрикциона.

	TCU.Gear = 3;
	TCU.GearChange = 0;	
}

static void gear_change_3_2() {
	TCU.GearChange = 1;

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_slu(get_slu_pressure_b3());
	loop_wait(GearChangeStep * 12);		// Ждем снижение давления в тормозе B2.

	SET_PIN_LOW(SOLENOID_S3_PIN);		// Отключаем систему "Clutch to Clutch".

	TCU.Gear = 2;
	TCU.GearChange = 0;	
}

/*
static void gear_change_3_1() {
	TCU.GearChange = 1;

	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	loop_wait(GearChangeStep * 6);	// Ждем повышения давления в гидроаккумуляторе.
	
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	loop_wait(GearChangeStep * 6);			// Ожидаем срабатывания фрикциона.

	TCU.Gear = 1;
	TCU.GearChange = 0;
}
*/
                                    
static void gear_change_2_1() {
	TCU.GearChange = 1;
	set_sln(get_sln_pressure()); 		// Соленоид SLN.
	loop_wait(GearChangeStep * 6);	// Ждем повышения давления в гидроаккумуляторе.
	
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}
	set_slu(SLU_MIN_VALUE);

	loop_wait(GearChangeStep * 6);			// Ожидаем срабатывания фрикциона.

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
	TCU.GearUpSpeed = get_gear_max_speed();		// Верхняя граница переключения.
	TCU.GearDownSpeed = get_gear_min_speed();	// Нижняя граница переключения.
}

// Контроль переключения передач, возвращает время задержки после переключения.
uint16_t gear_control() {
	// Только режимы D - L.
	if (TCU.ATMode < 4 || TCU.ATMode > 8) {return 0;}
	if (TCU.Gear < 1 || TCU.Gear > 5) {return 0;}

	set_gear_change_delays();	// Установка времени на переключение от ДПДЗ.

	get_gear_min_speed();	// Нижняя граница переключения.
	get_gear_max_speed();	// Верхняя граница переключения.

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
void slu_b3_control() {
	if (TCU.Gear != 2) {return;}	// Управление давлением SLU для второй передачи.
	set_slu(get_slu_pressure_b3());
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
	uint8_t ArraySize = sizeof(GearChangeStepArray) / sizeof(GearChangeStepArray[0]);
	GearChangeStep = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, GearChangeStepArray, ArraySize);
	
	ArraySize = sizeof(AfterChangeDelayArray) / sizeof(AfterChangeDelayArray[0]);
	AfterChangeDelay = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, AfterChangeDelayArray, ArraySize);
}

// Ожидание с основным циклом.
static void loop_wait(int16_t Delay) {
	WaitTimer = -1 * Delay;		// Устанавливаем время ожидания.
	while (WaitTimer < 0) {
		loop_main();
	}
}

static uint8_t get_gear_max_speed() {
	uint16_t* Array;
	uint8_t ArraySize;

	if (TCU.Gear == 5 || TCU.Gear <= 0) {return 130;}

	switch (TCU.Gear) {
		case 1:
			Array = Gear_1_2;
			ArraySize = sizeof(Gear_1_2) / sizeof(Gear_1_2[0]);
			break;
		case 2:
			Array = Gear_2_3;
			ArraySize = sizeof(Gear_2_3) / sizeof(Gear_2_3[0]);
			break;
		case 3:
			Array = Gear_3_4;
			ArraySize = sizeof(Gear_3_4) / sizeof(Gear_3_4[0]);
			break;
		case 4:
			Array = Gear_4_5;
			ArraySize = sizeof(Gear_4_5) / sizeof(Gear_4_5[0]);
			break;
		default:
			return 0;
	}
	return get_interpolated_value_uint16_t(TCU.TPS, TPSGrid, Array, ArraySize);
}

static uint8_t get_gear_min_speed() {
	if (TCU.Gear <= 1) {return 5;}

	uint16_t* Array;
	uint8_t ArraySize;

	switch (TCU.Gear) {
		case 2:
			Array = Gear_2_1;
			ArraySize = sizeof(Gear_2_1) / sizeof(Gear_2_1[0]);
			break;
		case 3:
			Array = Gear_3_2;
			ArraySize = sizeof(Gear_3_2) / sizeof(Gear_3_2[0]);
			break;
		case 4:
			Array = Gear_4_3;
			ArraySize = sizeof(Gear_4_3) / sizeof(Gear_4_3[0]);
			break;
		case 5:
			Array = Gear_5_4;
			ArraySize = sizeof(Gear_5_4) / sizeof(Gear_5_4[0]);
			break;
		default:
			return 0;
	}
	return get_interpolated_value_uint16_t(TCU.TPS, TPSGrid, Array, ArraySize);
}

static uint8_t rpm_after_ok(uint8_t Shift) {
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

// Функция проверяет, включилась ли передача.
// static uint8_t gear_check(uint8_t Gear) {
// 	// Расчетная скорость входного вала.
// 	uint16_t CalcDrumRPM = 0;

// 	switch (Gear) {
// 		case 1:
// 			CalcDrumRPM = ((uint32_t) TCU.OutputRPM * GEAR_1_RATIO) >> 10;
// 			break;
// 		case 2:
// 			CalcDrumRPM = ((uint32_t) TCU.OutputRPM * GEAR_2_RATIO) >> 10;
// 			break;
// 		case 3:
// 			CalcDrumRPM = ((uint32_t) TCU.OutputRPM * GEAR_3_RATIO) >> 10;
// 			break;
// 		case 4:
// 			CalcDrumRPM = ((uint32_t) TCU.OutputRPM * GEAR_4_RATIO) >> 10;
// 			break;
// 		default:
// 			return 1;
// 	}

// 	if (ABS(TCU.DrumRPM - CalcDrumRPM) < MAX_SLIP_RPM) {
// 		return 1;
// 	}
// 	else {
// 		return 0;
// 	}
// }

static void set_slt(uint8_t Value) {
	TCU.SLT = Value;
	OCR1A = TCU.SLT;	// SLT - выход A таймера 1.	
}

static void set_sln(uint8_t Value) {
	TCU.SLN = Value;
	OCR1B = TCU.SLN;	// SLN - выход B таймера 1.	
}

static void set_slu(uint8_t Value) {
	TCU.SLU = Value;
	OCR1C = TCU.SLU;	// SLU - выход C таймера 1.	
}
