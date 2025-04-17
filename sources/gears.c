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
uint16_t AfterChangeDelay = 1000;	// Пауза после включения передачи.

// Максимальная и минимальная передача для каждого режима.
//						  I  P   R  N  D  D4 D3 L2 L  E  M
//						  0  1   2  3  4  5  6  7  8  9  10
int8_t MaxGear[] = 		 {0, 0, -1, 0, 5, 4, 3, 2, 1, 0, 5};
const int8_t MinGear[] = {0, 0, -1, 0, 1, 1, 1, 2, 1, 0, 1};

uint8_t LastGear2ChangeTPS = 0;			// Значение ДПДЗ при последнем переключении 1>2.
uint8_t LastGear2ChangeSLU = 0;			// Значение SLU при последнем переключении 1>2.

uint8_t LastGear2ReactivateTPS = 0;		// Значение ДПДЗ при возобновлении второй передачи.
uint8_t LastGear2ReactivateRPM = 0;		// Значение опережения при возобновлении второй передачи.

uint8_t LastGear3ChangeTPS = 0;			// Значение ДПДЗ при последнем переключении 2>3.
uint16_t LastGear3ChangeSLU = 0;		// Задержка отключения SLU при последнем переключении 2>3.

uint8_t LastGear4ChangeTPS = 0;			// Значение ДПДЗ при последнем переключении 3>4.
uint8_t LastGear4ChangeSLT = 0;			// Значение SLT при последнем переключении 3>4.
uint8_t LastGear4ChangeSLN = 0;			// Значение SLN при последнем переключении 3>4.

int16_t LastPDRTime = 0;				// Последнее время включения PDR.
uint8_t Gear2LastStep = 0;

// Прототипы функций.
void loop_main(uint8_t Wait);		// Прототип функций из main.c.
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

static void slu_boost();
static void gear_change_wait(int16_t Delay, uint8_t Gear);

//static void set_slt(uint8_t Value);
static void set_sln(uint8_t Value);
static void set_slu(uint8_t Value);

//============================ Начальные передачи =============================
// Включение нейтрали.
void set_gear_n() {
	TCU.GearChange = -1;
	if (TCU.Gear < 0) {TCU.GearChange = 1;}

	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.
	set_sln(SLN_MIN_VALUE); 	// Минимальное давление в гидроаккумуляторах SLN.

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	TCU.Gear = 0;
	loop_wait(300);				// Пауза после включения нейтрали.

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

	loop_wait(500);

	TCU.Gear = 1;
	TCU.GearChange = 0;
}

// Включение задней передачи.
void set_gear_r() {
	TCU.GearChange = -1;

	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.
	set_sln(get_sln_pressure());

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	loop_wait(500);				// Пауза после включения передачи.

	TCU.Gear = -1;
	TCU.GearChange = 0;
}

// Блокировка задней передачи.
void disable_gear_r() {
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);	// S2 выключает привод.
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_slu(SLU_MIN_VALUE);
	set_sln(get_sln_pressure());
}

//=========================== Переключения вверх ==============================
static void gear_change_1_2() {
	#define GEAR_2_MAX_STEP 18

	TCU.GearChange = 1;
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_slu(get_slu_pressure_gear2());
	slu_boost();	// Первоначальная накачка давления SLU.

	LastGear2ChangeTPS = TCU.InstTPS;
	LastGear2ChangeSLU = TCU.SLU;

	uint8_t Step = 0;		// Шаг процесса включения передачи.
	uint8_t PDR = 0;		// Состояние процесса запроса снижения мощности.
	Gear2LastStep = GEAR_2_MAX_STEP;	// Номер последнего шага переключения.
	uint8_t SLUDelay = 0;	// Пауза повышения давления после начала переключния.
	int16_t PDRTime = 0;	// Для фиксации времени работы PDR.

	LastPDRTime = 0;
	if (TCU.InstTPS > 35) {PDR = -1;}
	while (Step < Gear2LastStep) {
		WaitTimer = -1 * GearChangeStep;				// Устанавливаем время ожидания.
		while (WaitTimer < 0) {
			loop_main(1);
			if (!PDR && rpm_delta(1) < -100) {			// Переключение началось.
				SLUDelay = 1;
				PDRTime = WaitTimer;
				SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрашиваем снижение мощности.
				PDR = 1;
			}
		}
		Step++;

		// Обороты валов выровнялись.
		if (ABS(rpm_delta(2)) < 20) {
			Gear2LastStep = Step + 2;
			if (Step <= 10) {
				// Передача включилась слишком рано,
				// снижаем давление на 1 единицу.
				save_gear2_adaptation(-1);
			}
		}

		if (Step > GEAR_2_MAX_STEP - 2 && rpm_delta(2) > 20) {
			// Передача включилась слишком поздно,
			// повышаем давление на 1 единицу.
			save_gear2_adaptation(1);
		}

		uint8_t NextSLU = get_slu_pressure_gear2() + Step / 2 - SLUDelay;
		// Прирост не более чем 1 единица за цикл.
		if (NextSLU > TCU.SLU) {set_slu(TCU.SLU + 1);}
		else {set_slu(NextSLU);}
	}

	if (PDR == 1) {LastPDRTime = WaitTimer - PDRTime;}
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	TCU.Gear = 2;
	TCU.GearChange = 0;
}

static void gear_change_2_3() {
	TCU.GearChange = 1;

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Включаем торможение двигателем в режиме "3".
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	set_sln(get_sln_pressure());
	//set_slu(get_slu_pressure_gear3());
	LastGear3ChangeTPS = TCU.InstTPS;
	LastGear3ChangeSLU = TCU.SLU;

	#define GEAR_3_MAX_STEP 18
	#define GEAR_3_SLU_ADD 10

	uint8_t Step = 0;
	while (Step < 18 && ABS(rpm_delta(3)) > 20) {
		WaitTimer = -1 * GearChangeStep;				// Устанавливаем время ожидания.
		while (WaitTimer < 0) {
			loop_main(1);
		}
		Step++;

		uint8_t CurrSLU = get_slu_pressure_gear2();
		int8_t Delta = GEAR_3_SLU_ADD - get_slu_pressure_add_gear3();
		int8_t Add = (int16_t) GEAR_3_SLU_ADD - Step * Delta / GEAR_3_MAX_STEP;
		set_slu(CurrSLU + 10 - Add);
	}

	set_slu(SLU_MIN_VALUE);
	TCU.Gear = 3;
	TCU.GearChange = 0;
}

static void gear_change_3_4() {
	TCU.GearChange = 1;

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	LastGear4ChangeTPS = TCU.InstTPS;
	LastGear4ChangeSLT = TCU.SLT;
	LastGear4ChangeSLN = TCU.SLN;

	set_sln(get_sln_pressure());
	
	gear_change_wait(GearChangeStep * 20, 4);

	TCU.Gear = 4;
	TCU.GearChange = 0;	
}

static void gear_change_4_5() {
	if (TCU.OilTemp < 30) {return;}
	TCU.GearChange = 1;

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_HIGH(SOLENOID_S4_PIN);

	set_sln(get_sln_pressure());
	
	glock_control(101);

	gear_change_wait(GearChangeStep * 20, 5);

	TCU.Gear = 5;
	TCU.GearChange = 0;	
}

//=========================== Переключения вниз ===============================
static void gear_change_5_4() {
	TCU.GearChange = -1;
	
	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_sln(get_sln_pressure());

	glock_control(101);
	loop_wait(GearChangeStep * 5);


	TCU.Gear = 4;
	TCU.GearChange = 0;		
}

static void gear_change_4_3() {
	TCU.GearChange = -1;
	
	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима 3. 
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	set_sln(get_sln_pressure());

	glock_control(101);
	loop_wait(GearChangeStep * 5);

	TCU.Gear = 3;
	TCU.GearChange = 0;	
}

static void gear_change_3_2() {
	TCU.GearChange = -1;

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);	// Включаем систему "Clutch to Clutch".
	SET_PIN_LOW(SOLENOID_S4_PIN);

	// Первоначальная накачка давления SLU.
	if (TCU.TPS > TPS_IDLE_LIMIT) {slu_boost();}

	// Реально вторая будет включаться далее в функции slu_gear2_control.
	// Это необходимо для контроля оборотов при включении второй передачи,
	// Чтобы не было резкого торможения двигателем.

	TCU.Gear = 2;
	TCU.GearChange = 0;	
}

static void gear_change_2_1() {
	TCU.GearChange = -1;

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	set_sln(get_sln_pressure());

	loop_wait(GearChangeStep * 5);
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
		return 100;
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
		return 100;
	}
	return 0;
}

/*
void slu_gear2_control(uint8_t Time) {
	static uint8_t Idle = 0;	// Флаг отключения передачи.

	if (TCU.Gear != 2) {
		Idle = 0;
		return;
	}

	if (TCU.TPS < TPS_IDLE_LIMIT) {
		Idle = 1;
		// В режимах "2" и "3", должно быть торможение двигателем,
		if (TCU.ATMode == 6 || TCU.ATMode == 7) {
			SET_PIN_LOW(SOLENOID_S3_PIN);
			set_slu(SLUGear2Graph[3]);	// SLU для торможения двигателем.
		}		
		else {	// В остальных случаях вторая передача на ХХ отключена.
			SET_PIN_HIGH(SOLENOID_S3_PIN);
			set_slu(SLU_MIN_VALUE);		// SLU для ХХ.
		}
		return;
	}

	int16_t Delta = rpm_delta(2);
	uint8_t CurrSLU = get_slu_pressure_gear2();
	if (Idle) {
		if (Delta >= -10) {
			// Обороты выровнялись, выход из режима ХХ.
			Idle = 0;
			SET_PIN_LOW(SOLENOID_S3_PIN);
			return;
		}

		if (Delta < -500) {
			// Ждем пасхи.
			if (TCU.ATMode == 6 || TCU.ATMode == 7) {set_slu(SLUGear2Graph[3]);}
			else {set_slu(SLU_MIN_VALUE);}
			return;
		}

		uint8_t SLUAdd = MAX(0, (int16_t) (7 + Delta / 64));
		set_slu(CurrSLU + SLUAdd);
	}
	else {
		set_slu(CurrSLU + 10);
	}
}
*/

void slu_gear2_control(uint8_t Time) {
	static uint16_t Timer = 0;		// Таймер правного включения.
	static uint8_t Step = 0;		// Шаги плавного включения.
	static uint16_t IdleTimer = 0;	// Таймер для ХХ.

	if (TCU.Gear != 2) {
		Timer = 0;
		Step = 0;
		IdleTimer = 0;
		return;
	}

	// чтобы при добавлении газа вторая передача плавно включилась.
	if (TCU.TPS < TPS_IDLE_LIMIT) {
		Timer = 0;
		Step = 0;
		IdleTimer++;
		// В режимах "2" и "3", должно быть торможение двигателем,
		if (TCU.ATMode == 6 || TCU.ATMode == 7) {
			SET_PIN_LOW(SOLENOID_S3_PIN);
			set_slu(SLUGear2Graph[3]);}		// SLU для торможения двигателем.
		else {	// В остальных случаях вторая передача на ХХ отключена.
			SET_PIN_HIGH(SOLENOID_S3_PIN);
			set_slu(SLU_MIN_VALUE);		// SLU для ХХ.
		}
		return;
	}

	// Проверка оборотов перед включением.
	// Вторая передача не должна включаться слишком рано.
	int16_t MinDelta = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, Gear2DeltaRPM, TPS_GRID_SIZE);
	int16_t Delta = rpm_delta(2);
	if (Delta < -1 * MinDelta) {
		Timer = 0;
		Step = 0;
		IdleTimer++;

		SET_PIN_HIGH(SOLENOID_S3_PIN);
		set_slu(SLU_MIN_VALUE);
		return;
	}

	uint8_t SLUDelay = 0;	// Пауза при выравнивании оборотов.
	if (Delta < -10 && Delta < -250) {SLUDelay = 1;}

	if (TCU.SLU <= SLUGear2Graph[0]) {set_slu(get_slu_pressure_gear2());}

	if (IdleTimer > 60) {slu_boost();}
	IdleTimer = 0;

	// Плавное включение второй передачи.
	if (Step < 10) {
		Timer += Time;
		if (Timer > GearChangeStep * 2) {
			Timer = 0;
			Step++;
		}
		if (Step == 8) {SET_PIN_LOW(SOLENOID_S3_PIN);}
	}
	else {
		SLUDelay = 0;
	}

	uint8_t NextSLU = get_slu_pressure_gear2() + Step + SLUDelay;
	// Прирост не более чем 1 единица за цикл.
	if (NextSLU > TCU.SLU) {set_slu(TCU.SLU + 1);}
	else {set_slu(NextSLU);}
}


// Первоначальная накачка давления SLU для включения второй передачи.
static void slu_boost() {
	//return;
	// Ограничение по ДПДЗ и температуре масла.
	//if (TCU.OilTemp < 25) {return;}
	if (TCU.InstTPS > 35) {return;}
	
	// Ограничение по начальному давлению.
	if (TCU.SLU > SLUGear2Graph[0]) {return;}

	uint8_t Add = 20;
	if (TCU.SLU <= SLU_MIN_VALUE) {Add = 25;}

	uint8_t CurrSLU = TCU.SLU;
	set_slu(MIN(230, CurrSLU + Add));
	loop_wait(SOLENOID_BOOST_TIME);
	set_slu(CurrSLU);
}

static void gear_change_wait(int16_t Delay, uint8_t Gear) {
	uint8_t PDR = 0;
	int16_t PDRTime = 0;	// Для фиксации времени работы PDR.
	if (TCU.InstTPS > 35) {PDR = -1;}
	LastPDRTime = 0;
	WaitTimer = -1 * Delay;		// Устанавливаем время ожидания.
	while (WaitTimer < 0 && rpm_delta(Gear) > 20) {
		if (!PDR && rpm_delta(Gear - 1) < -100) {	// Переключение началось.
			PDRTime = WaitTimer;
			SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);
			PDR = 1;
		}
		loop_main(1);
	}
	loop_wait(200);
	if (PDR == 1) {LastPDRTime = WaitTimer - PDRTime;}
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);
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
	while (WaitTimer < 0) {loop_main(1);}
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
