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
int8_t MaxGear[] = {0, 0, -1, 0, 5, 4, 3, 2, 1, 0, 5};
const int8_t MinGear[] = {0, 0, -1, 0, 1, 1, 1, 2, 1, 0, 1};

uint8_t LastGear2ChangeTPS = 0;			// Значение ДПДЗ при последнем переключении 1>2.
uint8_t LastGear2ChangeSLU = 0;			// Значение SLU при последнем переключении 1>2.

uint8_t LastGear3ChangeTPS = 0;			// Значение ДПДЗ при последнем переключении 2>3.
uint16_t LastGear3ChangeSLUDelay = 0;	// Задержка отключения SLU при последнем переключении 2>3.

uint8_t LastGear4ChangeTPS = 0;			// Значение ДПДЗ при последнем переключении 3>4.
uint8_t LastGear4ChangeSLT = 0;			// Значение SLT при последнем переключении 3>4.
uint8_t LastGear4ChangeSLN = 0;			// Значение SLN при последнем переключении 3>4.

// Прототипы функций.
void loop_main();					// Прототип функций из main.c.
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
	TCU.GearChange = 1;

	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.
	set_sln(SLN_MIN_VALUE); 	// Минимальное давление в гидроаккумуляторах SLN.

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	TCU.Gear = 0;
	loop_wait(250);				// Пауза после включения нейтрали.
	TCU.GearChange = 0;
}

// Включение первой передачи.
void set_gear_1() {
	TCU.GearChange = 1;

	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.
	set_sln(get_sln_pressure());

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}
	TCU.Gear = 1;

	loop_wait(250);				// Пауза после включения передачи.
	set_sln(SLN_MIN_VALUE);

	TCU.GearChange = 0;
}

// Включение задней передачи.
void set_gear_r() {
	TCU.GearChange = 1;

	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.
	set_sln(get_sln_pressure());

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	TCU.Gear = -1;
	loop_wait(250);				// Пауза после включения передачи.
	set_sln(SLN_MIN_VALUE);

	TCU.GearChange = 0;
}

// Блокировка задней передачи.
void disable_gear_r() {
	set_slu(SLU_MIN_VALUE);
	set_sln(SLN_MIN_VALUE); 		// Минимальное давление в гидроаккумуляторах SLN.
	
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);	// S2 выключает привод.
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
}

//=========================== Переключения вверх ==============================
static void gear_change_1_2() {
	TCU.GearChange = 1;

	// Начальная накачка давления.
	set_sln(SOLENOID_BOOST_VALUE);
	set_slu(SOLENOID_BOOST_VALUE);
	loop_wait(SOLENOID_BOOST_TIME);

	set_sln(get_sln_pressure());
	set_slu(get_slu_pressure_gear2());		// Давление включения и работы второй предачи.
	LastGear2ChangeTPS = TCU.InstTPS;
	LastGear2ChangeSLU = TCU.SLU;

	loop_wait(GearChangeStep * 5);
	set_slu(get_slu_pressure_gear2());

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);			// Включаем систему "Clutch to Clutch".
	SET_PIN_LOW(SOLENOID_S4_PIN);
	TCU.Gear = 2;

	set_sln(SLN_MIN_VALUE);

	for (uint8_t i = 0; i < 4 * 3; ++i) {
		loop_wait(GearChangeStep);
		slu_gear2_control(GearChangeStep + 1);
	}

	TCU.GearChange = 0;
}

static void gear_change_2_3() {
	TCU.GearChange = 1;

	// Начальная накачка давления.
	set_sln(SOLENOID_BOOST_VALUE);
	loop_wait(SOLENOID_BOOST_TIME);

	set_sln(get_sln_pressure());

	// Задержка отключения давления SLU при включением третьей передачи.
	uint16_t SLUDelay = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, Gear3SLUDelayGraph, TPS_GRID_SIZE);

	LastGear3ChangeTPS = TCU.InstTPS;
	LastGear3ChangeSLUDelay = SLUDelay;

	SET_PIN_LOW(SOLENOID_S3_PIN);			// Отключаем систему "Clutch to Clutch".

	loop_wait(GearChangeStep * 5);			// Ждем повышения давления.
	set_slu(get_slu_pressure_gear2());
	set_sln(SLN_MIN_VALUE);

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	TCU.Gear = 3;

	loop_wait(SLUDelay);
	set_slu(SLU_MIN_VALUE);

	// Включаем торможение двигателем в режиме "3".
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}
	
	TCU.GearChange = 0;
}

static void gear_change_3_4() {
	TCU.GearChange = 1;

	set_sln(get_sln_pressure());

	LastGear4ChangeTPS = TCU.InstTPS;
	LastGear4ChangeSLT = TCU.SLT;
	LastGear4ChangeSLN = TCU.SLN;

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	TCU.Gear = 4;

	loop_wait(GearChangeStep * 13);
	set_sln(SLN_MIN_VALUE);

	TCU.GearChange = 0;	
}

static void gear_change_4_5() {
	TCU.GearChange = 1;
	glock_control(100);				// Снижаем давление блокировки гидротрансформатора.

	set_sln(get_sln_pressure());

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_HIGH(SOLENOID_S4_PIN);
	TCU.Gear = 5;

	loop_wait(GearChangeStep * 13);
	set_sln(SLN_MIN_VALUE);

	TCU.GearChange = 0;	
}

//=========================== Переключения вниз ===============================
static void gear_change_5_4() {
	TCU.GearChange = 1;
	glock_control(100);	// Снижаем давление блокировки гидротрансформатора.
	
	set_sln(get_sln_pressure());

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	TCU.Gear = 4;

	loop_wait(GearChangeStep * 8);
	set_sln(SLN_MIN_VALUE);

	TCU.GearChange = 0;		
}

static void gear_change_4_3() {
	TCU.GearChange = 1;
	glock_control(100);	// Снижаем давление блокировки гидротрансформатора.
	
	set_sln(get_sln_pressure());

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима 3. 
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}
	TCU.Gear = 3;

	loop_wait(GearChangeStep * 8);
	set_sln(SLN_MIN_VALUE);

	TCU.GearChange = 0;	
}

static void gear_change_3_2() {
	TCU.GearChange = 1;

	set_slu(SLU_MIN_VALUE);			// Давление минимум.
	loop_wait(GearChangeStep * 4);

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".
	SET_PIN_LOW(SOLENOID_S4_PIN);
	TCU.Gear = 2;

	// Реально вторая будет включаться далее в функции slu_gear2_control.
	// Это необходимо для контроля оборотов при включении второй передачи,
	// Что не было резкого торможения двигателем.

	TCU.GearChange = 0;	
}

static void gear_change_2_1() {
	TCU.GearChange = 1;

	set_sln(get_sln_pressure());
	
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}
	TCU.Gear = 1;

	loop_wait(GearChangeStep * 12);
	set_slu(SLU_MIN_VALUE);
	set_sln(SLN_MIN_VALUE);

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
		return 100;
	} 
	if (TCU.Gear < MinGear[TCU.ATMode]) {
		// Проверка оборотов просле переключения.
		if (rpm_after_ok(1)) {gear_up();}
		return AfterChangeDelay;
	}

	if (TCU.InstTPS > TPS_WOT_LIMIT) {
		// Переключение по оборотам.
		if (TCU.DrumRPM > GearUpRPM[TCU.Gear]) {
			gear_up();
			return GearUpDelay[TCU.Gear];
		}
		if (TCU.DrumRPM < GearDownRPM[TCU.Gear]) {
			gear_up();
			return GearDownDelay[TCU.Gear];
		}
	}
	else {
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
			return 100;
		}
	}
	return 0;
}

// Управление давлением SLU B3 для работы второй передачи,
// а также управление добавочным соленоидом S3
void slu_gear2_control(uint8_t Time) {
	static int16_t Timer = 0;	// Таймер правного включения.
	static int8_t Step = 0;		// Шгаги плавного включения
	static uint8_t AfterlIdle = 0;		// Уменьшения давления после ХХ.
	if (TCU.Gear != 2) {	// Управление давлением SLU для второй передачи.
		Timer = 0;
		Step = 0;
		AfterlIdle = 0;
		return;
	}

	// Включаем систему "Clutch to Clutch" на ХХ,
	// чтобы при добавлении газа вторая передача плавно включилась.
	if (TCU.TPS < TPS_IDLE_LIMIT) {
		Timer = 0;
		Step = 0;
		SET_PIN_HIGH(SOLENOID_S3_PIN);

		// В режимах "2" и "3", должно быть торможение двигателем,
		// в остальных случаях вторая передача на ХХ отключена.
		if (TCU.ATMode == 6 || TCU.ATMode == 7) {set_slu(SLUGear2Graph[2]);}
		else {
			set_slu(SLU_MIN_VALUE);
			AfterlIdle = 1;
		}
		return;
	}

	// Проверка оборотов перед включением.
	// Вторая передача не должна включаться слишком рано.
	uint16_t NewRPM = ((uint32_t) TCU.OutputRPM * GEAR_2_RATIO) >> 10;
	uint16_t Delta = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, Gear2DeltaRPM, TPS_GRID_SIZE);
	if (NewRPM > TCU.DrumRPM && NewRPM - TCU.DrumRPM > Delta) {
		Timer = 0;
		Step = 0;

		SET_PIN_HIGH(SOLENOID_S3_PIN);
		set_slu(SLU_MIN_VALUE);
		AfterlIdle = 1;
		return;
	}



	// Плавное включение второй передачи после отключения.
	if (Step < 7) {
		Timer += Time;
		if (Step > 4) {Timer += Time;}	// Ускорение срабатывания.

		if (Timer > GearChangeStep * 3) {
			Timer = 0;
			Step++;
		}

		if (AfterlIdle && TCU.SLU <= SLU_MIN_VALUE) {
			set_slu(SOLENOID_BOOST_VALUE);
			loop_wait(SOLENOID_BOOST_TIME);
			AfterlIdle = 0;
		}

		set_slu(get_slu_pressure_gear2() + Step);
		return;
	}
	else {
		SET_PIN_LOW(SOLENOID_S3_PIN);
		return;
	}

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
// 	OCR1A = TCU.SLT;	// SLN - выход A таймера 1.
// }

static void set_sln(uint8_t Value) {
	TCU.SLN = Value;
	OCR1B = TCU.SLN;	// SLN - выход B таймера 1.	
}

static void set_slu(uint8_t Value) {
	TCU.SLU = Value;
	OCR1C = TCU.SLU;	// SLU - выход C таймера 1.	
}
