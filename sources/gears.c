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
uint16_t GearChangeTime = 0;		// Время на переключение передачи.
uint16_t AfterChangeDelay = 0;		// Задержка после переключения.

// Максимальная и минимальная передача для каждого режима.
//						  I  P   R  N  D  D4 D3 L2 L  E  M
//						  0  1   2  3  4  5  6  7  8  9  10
const int8_t MaxGear[] = {0, 0, -1, 0, 5, 4, 3, 2, 1, 0, 5};
const int8_t MinGear[] = {0, 0, -1, 0, 1, 1, 1, 1, 1, 0, 1};

//								1   2   3   4  5
const int8_t MinGearSpeed[5] = {0,  11, 22, 38, 75};
const int8_t MaxGearSpeed[5] = {15, 26, 44, 79, 255};

// Прототипы функций.
void loop_main();	// Прототип функций из main.c.

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
static void loop_wait(int16_t Delay);
static uint8_t rpm_after_ok(uint8_t Shift);
static void set_slt(uint8_t Value);
static void set_sln(uint8_t Value);
static void set_slu(uint8_t Value);

//============================ Начальные передачи =============================
// Включение нейтрали.
void set_gear_n() {
	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.

	TCU.GearChange = 1;
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_sln(get_sln_value()); 	// Соленоид SLN.

	TCU.Gear = 0;
	TCU.GearChange = 0;

	loop_wait(500);				// Пауза после включения нейтрали.
}

// Включение первой передачи.
void set_gear_1() {
	set_slu(SLU_MIN_VALUE);
	set_slt(SLT_START_VALUE);
	TCU.GearChange = 1;

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	// Ждем 500 мс.
	for (uint8_t i = 0; i < 25; i++) {
		set_sln(MAX(SLN_MIN_VALUE, TCU.SLN - 4));
		loop_wait(50);
	}
	set_sln(SLN_MIN_VALUE); 	// Соленоид SLN минимум.

	TCU.Gear = 1;
	TCU.GearChange = 0;
}

// Включение задней передачи.
void set_gear_r() {
	set_slu(SLU_MIN_VALUE);
	set_slt(SLT_START_VALUE);
	TCU.GearChange = 1;
	
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	// Ждем 500 мс.
	for (uint8_t i = 0; i < 25; i++) {
		set_sln(MAX(SLN_MIN_VALUE, TCU.SLN - 4));
		loop_wait(50);
	}

	TCU.Gear = -1;
	TCU.GearChange = 0;
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
	// Находим стартовое давление тормоза B3 (SLU.)
	uint8_t ArraySize = sizeof(StartPressureB3Array) / sizeof(StartPressureB3Array[0]);
	uint8_t StartPressureB3 = get_interpolated_value_uint8_t(TCU.TPS, StartPressureB3Array, ArraySize);
	
	// Применяем коррекцию по температуре.
	ArraySize = sizeof(B3TempCorrGraph) / sizeof(B3TempCorrGraph[0]);
	int8_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, TempGridB3, B3TempCorrGraph, ArraySize);
	StartPressureB3 += OilTempCorr;

	TCU.GearChange = 1;
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.
	
	set_slu(StartPressureB3);		// Порог схватывания фрикциона.
	loop_wait(GearChangeTime);		// Ждем повышения давления в тормозе B3.

	// Плавно добавляем значение SLU.
	uint8_t Delay = GearChangeTime / 2;
	for (uint8_t i = 0; i < 2; i++) {
		set_slu(MIN(255, TCU.SLU + 1));
		loop_wait(Delay);
	}
	set_slu(SLU_B3_WORK);	// Рабочее давление второй передача (B3) от SLU.
	
	TCU.Gear = 2;
	TCU.GearChange = 0;
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	loop_wait(AfterChangeDelay);	// Пауза после включения передачи.
}

static void gear_change_2_3() {
	TCU.GearChange = 1;
	set_sln(get_sln_value()); 	// Соленоид SLN.
	loop_wait(GearChangeTime / 2);	// Ждем повышения давления в гидроаккумуляторе.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);		// Ускоряет включение передачи.
	SET_PIN_LOW(SOLENOID_S4_PIN);

	// Отличие для режима 3. 
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.

	loop_wait(GearChangeTime);		// Ожидаем срабатывание фрикциона.
	set_slu(SLU_MIN_VALUE);				// Отключаем давление на B3.

	// Плавно снижаем давление в гидроаккумуляторе и SLU.
	uint8_t Delay = GearChangeTime / 10;
	for (uint8_t i = 0; i < 10; i++) {
		set_sln(MAX(SLN_MIN_VALUE, TCU.SLN - 4));
		loop_wait(Delay);
	}	

	TCU.Gear = 3;
	TCU.GearChange = 0;
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	loop_wait(AfterChangeDelay);	// Пауза после включения передачи.
}

static void gear_change_3_4() {
	TCU.GearChange = 1;
	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(GearChangeTime / 2);	// Ждем повышения давления в гидроаккумуляторе.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.

	loop_wait(GearChangeTime / 2);		// Ожидаем срабатывание фрикциона.
	// Плавно снижаем давление в гидроаккумуляторе.
	uint8_t Delay = GearChangeTime / 10;
	for (uint8_t i = 0; i < 10; i++) {
		set_sln(MAX(SLN_MIN_VALUE, TCU.SLN - 4));
		loop_wait(Delay);
	}	
	set_sln(SLN_MIN_VALUE);		// Соленоид SLN минимум.

	TCU.Gear = 4;
	TCU.GearChange = 0;	
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	loop_wait(AfterChangeDelay);	// Пауза после включения передачи.
}

static void gear_change_4_5() {
	TCU.GearChange = 1;
	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(GearChangeTime / 2);	// Ждем повышения давления в гидроаккумуляторе.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_HIGH(SOLENOID_S4_PIN);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.

	loop_wait(GearChangeTime / 2);		// Ожидаем срабатывание фрикциона.
	// Плавно снижаем давление в гидроаккумуляторе.
	uint8_t Delay = GearChangeTime / 10;
	for (uint8_t i = 0; i < 10; i++) {
		set_sln(MAX(SLN_MIN_VALUE, TCU.SLN - 4));
		loop_wait(Delay);
	}	
	set_sln(SLN_MIN_VALUE);		// Соленоид SLN минимум.

	TCU.Gear = 5;
	TCU.GearChange = 0;	
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	loop_wait(AfterChangeDelay);	// Пауза после включения передачи.
}

//=========================== Переключения вниз ===============================
static void gear_change_5_4() {
	TCU.GearChange = 1;
	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(GearChangeTime / 2);	// Ждем повышения давления в гидроаккумуляторе.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	loop_wait(GearChangeTime / 2);		// Ожидаем срабатывание фрикциона.
	// Плавно снижаем давление в гидроаккумуляторе.
	uint8_t Delay = GearChangeTime / 10;
	for (uint8_t i = 0; i < 10; i++) {
		set_sln(MAX(SLN_MIN_VALUE, TCU.SLN - 4));
		loop_wait(Delay);
	}	
	set_sln(SLN_MIN_VALUE);		// Соленоид SLN минимум.
	TCU.Gear = 4;
	TCU.GearChange = 0;		

	loop_wait(AfterChangeDelay);	// Пауза после включения передачи.
}

static void gear_change_4_3() {
	TCU.GearChange = 1;
	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(GearChangeTime / 2);	// Ждем повышения давления в гидроаккумуляторе.

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима 3. 
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	loop_wait(GearChangeTime / 2);		// Ожидаем срабатывание фрикциона.
	// Плавно снижаем давление в гидроаккумуляторе.
	uint8_t Delay = GearChangeTime / 10;
	for (uint8_t i = 0; i < 10; i++) {
		set_sln(MAX(SLN_MIN_VALUE, TCU.SLN - 4));
		loop_wait(Delay);
	}	
	set_sln(SLN_MIN_VALUE);		// Соленоид SLN минимум.
	TCU.Gear = 3;
	TCU.GearChange = 0;	

	loop_wait(AfterChangeDelay);	// Пауза после включения передачи.
}

static void gear_change_3_2() {
	// Находим стартовое давление тормоза B3 (SLU.)
	uint8_t ArraySize = sizeof(StartPressureB3Array) / sizeof(StartPressureB3Array[0]);
	uint8_t StartPressureB3 = get_interpolated_value_uint8_t(TCU.TPS, StartPressureB3Array, ArraySize);

	// Применяем коррекцию по температуре.
	ArraySize = sizeof(B3TempCorrGraph) / sizeof(B3TempCorrGraph[0]);
	int8_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, TempGridB3, B3TempCorrGraph, ArraySize);
	StartPressureB3 += OilTempCorr;
	
	TCU.GearChange = 1;
	set_slu(StartPressureB3 - 1);

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем клапан сброса давления B2.
	SET_PIN_LOW(SOLENOID_S4_PIN);

	loop_wait(GearChangeTime);		// Ждем снижение давления в тормозе B2.
	set_slu(StartPressureB3);
	loop_wait(GearChangeTime);		// Ждем повышения давления в тормозе B3.
	SET_PIN_LOW(SOLENOID_S3_PIN);	// Выключаем клапан сброса давления B2.

	// Плавно добавляем значение SLU.
	uint8_t Delay = GearChangeTime / 2;
	for (uint8_t i = 0; i < 2; i++) {
		set_slu(MIN(255, TCU.SLU + 1));
		loop_wait(Delay);
	}
	set_slu(SLU_B3_WORK);		// Рабочее давление второй передача (B3) от SLU.

	TCU.Gear = 2;
	TCU.GearChange = 0;	
	loop_wait(AfterChangeDelay);	// Пауза после включения передачи.
}
                                    
static void gear_change_2_1() {
	TCU.GearChange = 1;
	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(GearChangeTime / 2);	// Ждем повышения давления в гидроаккумуляторе.
	
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}
	set_slu(SLU_MIN_VALUE);

	loop_wait(GearChangeTime / 2);		// Ожидаем срабатывание фрикциона.
	// Плавно снижаем давление в гидроаккумуляторе.
	uint8_t Delay = GearChangeTime / 10;
	for (uint8_t i = 0; i < 10; i++) {
		set_sln(MAX(SLN_MIN_VALUE, TCU.SLN - 4));
		loop_wait(Delay);
	}	
	set_sln(SLN_MIN_VALUE);		// Соленоид SLN минимум.
	TCU.Gear = 1;
	TCU.GearChange = 0;

	loop_wait(AfterChangeDelay);	// Пауза после включения передачи.
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

// Контроль переключения передач.
void gear_control() {
	// Только режимы D - L.
	if (TCU.ATMode < 4 || TCU.ATMode > 8) {return;}

	uint8_t ArraySize = sizeof(GearChangeTimeArray) / sizeof(GearChangeTimeArray[0]);
	GearChangeTime = get_interpolated_value_uint16_t(TCU.TPS, GearChangeTimeArray, ArraySize);
	
	ArraySize = sizeof(AfterChangeDelayArray) / sizeof(AfterChangeDelayArray[0]);
	AfterChangeDelay = get_interpolated_value_uint16_t(TCU.TPS, AfterChangeDelayArray, ArraySize);

	if (TCU.Gear < 1 || TCU.Gear > 5) {return;}
	// uint8_t MinSpeed = MinGearSpeed[TCU.Gear - 1];
	// uint8_t MaxSpeed = MaxGearSpeed[TCU.Gear - 1];
	// TCU.GearDownSpeed = MinSpeed; //get_gear_min_speed();	// Нижняя граница переключения.
	// TCU.GearUpSpeed = MaxSpeed; //get_gear_max_speed();		// Верхняя граница переключения.

	get_gear_min_speed();	// Нижняя граница переключения.
	get_gear_max_speed();	// Верхняя граница переключения.

	// Переключения при изменение режима АКПП.
	if (TCU.Gear > MaxGear[TCU.ATMode]) {	
		// Проверка оборотов просле переключения.
		if (rpm_after_ok(-1)) {gear_down();}
		return;
	} 
	if (TCU.Gear < MinGear[TCU.ATMode]) {
		// Проверка оборотов просле переключения.
		if (rpm_after_ok(1)) {gear_up();}
		return;
	}

	// Скорость выше порога.
	if (TCU.CarSpeed > MaxSpeed) {
		if (TCU.InstTPS > 2) {gear_up();}	// Не повышать передачу при сбросе газа.
		return;
	}

	// Скорость ниже порога.
	if (TCU.CarSpeed < MinSpeed) {
		gear_down();
		return;
	}
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

// Ожидание с основным циклом.
static void loop_wait(int16_t Delay) {
	WaitTimer = -1 * Delay;		// Устанавливаем время ожидания.
	while (WaitTimer < 0) {
		loop_main();
	}
}

uint8_t get_gear_max_speed() {
	uint8_t* Array;
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
	return get_interpolated_value_uint8_t(TCU.TPS, Array, ArraySize);
}

uint8_t get_gear_min_speed() {
	if (TCU.Gear <= 1) {return 8;}

	uint8_t* Array;
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
	return get_interpolated_value_uint8_t(TCU.TPS, Array, ArraySize);
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
