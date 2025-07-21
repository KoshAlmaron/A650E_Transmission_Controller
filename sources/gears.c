#include <stdint.h>			// Коротние название int.
#include <avr/io.h>			// Названия регистров и номера бит.
#include <avr/interrupt.h>	// Прерывания.

#include "gears.h"			// Свой заголовок.
#include "gears_tables.h"	// Таблицы скоростей переключения передач.
#include "pinout.h"			// Список назначенных выводов.
#include "macros.h"			// Макросы.
#include "mathemat.h"		// Математические функции.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "configuration.h"	// Настройки.
#include "spdsens.h"		// Датчики скорости валов.

extern uint16_t WaitTimer;			// Таймер ожидания из main.
uint16_t GearChangeStep = 100;		// Шаг времени на переключение передачи.

// Максимальная и минимальная передача для каждого режима.
//					I  P   R  N  D  D4 D3 L2 L  E  M
//					0  1   2  3  4  5  6  7  8  9  10
int8_t MaxGear[] = {0, 0, -1, 0, 5, 4, 3, 2, 1, 0, 5};
int8_t MinGear[] = {0, 0, -1, 0, 1, 1, 1, 2, 1, 0, 1};

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
static void loop_wait(uint16_t Delay);

static uint8_t rpm_after_ok(uint8_t Shift);

static void slu_boost();
static void gear_change_wait(uint16_t Delay, uint8_t Gear);

//static void set_slt(uint8_t Value);
static void set_sln(uint16_t Value);
static void set_slu(uint16_t Value);

//============================ Начальные передачи =============================
// Включение нейтрали.
void set_gear_n() {
	TCU.GearChange = -1;
	if (TCU.Gear < 0) {TCU.GearChange = 1;}

	set_slu(SLU_MIN_VALUE);			// Выключение SLU на случай переключения со второй передачи.
	TCU.Gear2State = 0;
	set_sln(SLN_MIN_PRESSURE); 

	TCU.Gear = 0;
	loop_wait(700);				// Пауза после включения нейтрали.
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_sln(SLN_IDLE_PRESSURE); 

	TCU.GearChange = 0;
}

// Включение первой передачи.
void set_gear_1() {
	TCU.GearChange = 1;

	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.
	set_sln(get_sln_pressure());
	loop_wait(200);

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);		

	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}
	loop_wait(1300);
	
	set_sln(SLN_IDLE_PRESSURE);

	TCU.Gear = 1;
	TCU.GearChange = 0;
}

// Включение задней передачи.
void set_gear_r() {
	TCU.GearChange = -1;

	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.
	TCU.Gear2State = 0;
	set_sln(get_sln_pressure());
	loop_wait(200);

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	loop_wait(1400);

	set_sln(SLN_IDLE_PRESSURE);

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
	set_sln(SLN_IDLE_PRESSURE);
}

//=========================== Переключения вверх ==============================
static void gear_change_1_2() {
	#define GEAR_2_MAX_STEP 20
	#define GEAR_2_STEP_ADD 2

	TCU.GearChange = 1;
	slu_boost();						// Первоначальная накачка давления SLU.

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".
	SET_PIN_LOW(SOLENOID_S4_PIN);

	uint16_t NextSLU = get_slu_pressure_gear2();
	set_slu(NextSLU);

	TCU.GearChangeTPS = TCU.InstTPS;
	TCU.GearChangeSLU = TCU.SLU;

	set_sln(SLN_MIN_PRESSURE);

	TCU.GearStep = 0;		// Шаг процесса включения передачи.
	uint8_t PDR = 0;		// Состояние процесса запроса снижения мощности.
	uint8_t PDRStep = 0;	// Для фиксации времени работы PDR.
	TCU.LastStep = GEAR_2_MAX_STEP;	// Номер последнего шага переключения.
	uint8_t SLUDelay = 0;	// Пауза повышения давления после начала переключния.
	int8_t Adaptation = 0;	// Флаг применения адаптции.
	uint16_t SLUTimer = 0;
	TCU.LastPDRTime = 0;
	if (TCU.InstTPS > PDR_MAX_TPS) {PDR = -1;}
	
	while (TCU.GearStep < TCU.LastStep) {
		WaitTimer = GearChangeStep;				// Устанавливаем время ожидания.
		SLUTimer = GearChangeStep;
		while (WaitTimer) {
			loop_main(1);

			// Изменение давления из-за изменения значения ДПДЗ.
			if (SLUTimer - WaitTimer >= 25) {
				SLUTimer = WaitTimer;
				NextSLU = get_slu_pressure_gear2() + TCU.GearStep * 2 - SLUDelay;
				if (NextSLU > TCU.SLU) {
					// Прирост не более чем 8 единиц за цикл.
					if (NextSLU - TCU.SLU <= 8) {set_slu(NextSLU);}
					else {set_slu(TCU.SLU + 8);}
				}
				else {set_slu(NextSLU);}
			}

			if (!PDR && rpm_delta(1) < -75) {			// Переключение началось.
				PDR = 1;
				SLUDelay = 4;
				PDRStep = TCU.GearStep;
				SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрашиваем снижение мощности.
			}
		}
		TCU.GearStep++;

		// Обороты валов выровнялись.
		if (ABS(rpm_delta(2)) < 30) {
			if (PDR == 1) {TCU.LastPDRTime = (TCU.GearStep - PDRStep) * GearChangeStep;}
			SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

			//if (TCU.LastStep == GEAR_2_MAX_STEP) {TCU.LastStep = TCU.GearStep + GEAR_2_STEP_ADD;}
			if (TCU.GearStep < 13 && !Adaptation) {
				// Передача включилась слишком рано,
				// снижаем давление на 1 единицу.
				Adaptation = -1;
				save_gear2_adaptation(-1);
			}
		}
		if (TCU.GearStep > 15 && rpm_delta(2) > 30 && !Adaptation) {
			// Передача включилась слишком поздно,
			// повышаем давление на 1 единицу.
			Adaptation = 1;
			save_gear2_adaptation(1);
		}

		// Изменение давления по циклу.
		NextSLU = get_slu_pressure_gear2() + TCU.GearStep * 2 - SLUDelay;
		if (NextSLU > TCU.SLU) {
			// Прирост не более чем 8 единицы за цикл.
			if (NextSLU - TCU.SLU <= 8) {set_slu(NextSLU);}
			else {set_slu(TCU.SLU + 8);}
		}
		else {set_slu(NextSLU);}
	}

	TCU.LastStep -= GEAR_2_STEP_ADD;
	SET_PIN_LOW(SOLENOID_S3_PIN);
	set_sln(SLN_IDLE_PRESSURE);

	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);
	
	TCU.Gear = 2;
	TCU.Gear2State = 8;
	TCU.GearChange = 0;
}

static void gear_change_2_3() {
	TCU.GearChange = 1;
	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_slu(get_slu_pressure_gear3());
	TCU.GearChangeTPS = TCU.InstTPS;
	TCU.GearChangeSLU = TCU.SLU;

	TCU.LastStep = 0;
	set_sln(SLN_IDLE_PRESSURE);

	WaitTimer = get_gear3_slu_delay(TCU.InstTPS);
	int16_t SLNOffset = get_gear3_sln_offset(TCU.InstTPS);
	uint8_t SetSLN = 0;
	// Ждем начало включения B2.
	while (WaitTimer) {
		loop_main(1);

		// Давление SLU включения третьей передачи
		set_slu(get_slu_pressure_gear3());
		
		if (!SetSLN) {
			SLNOffset = get_gear3_sln_offset(TCU.InstTPS);
		 	if (WaitTimer + SLNOffset < 5) {SetSLN = 1;}	// Пересечение SNL и SLU при SLNOffset < 0.
		}
		else {set_sln(get_sln_pressure_gear3());}
	}

	set_slu(SLU_MIN_VALUE);		// Убираем давление SLU.

	uint8_t PDR = 0;
	uint16_t PDRTime = 0;
	uint8_t Gear2Slip = 0;	// Флаг наличия проскальзывания.
	if (TCU.InstTPS > PDR_MAX_TPS) {PDR = -1;}

	// Ждем включения третьей передачи.
	WaitTimer = 1500;
	SLNOffset = WaitTimer - SLNOffset;
	while (WaitTimer && rpm_delta(3) > 30) {
		loop_main(1);
		if (rpm_delta(2) < -100) {		// Переключение началось.
			if (!PDR) {
				PDR = 1;
				PDRTime = WaitTimer;
				SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);
			}
		}

		if (SLNOffset >= WaitTimer) {			// Задержка SLN при SLNOffset > 0.
			set_sln(get_sln_pressure_gear3());	// Устанавливаем давление SLN.
		}
		if (!Gear2Slip && rpm_delta(2) > 120) {
			// Проскальзывание второй передачи.
			// Выключение произошло слишком рано.
			Gear2Slip = 1;
		}
	}

	TCU.GearChangeSLN = TCU.SLN;
	if (Gear2Slip) {TCU.LastStep = 99;}
	set_sln(SLN_IDLE_PRESSURE);

	if (PDR == 1) {
		TCU.LastPDRTime = PDRTime - WaitTimer + 200;
		loop_wait(200);
	}
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	// Включаем торможение двигателем в режиме "3".
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	TCU.Gear = 3;
	TCU.Gear2State = 0;
	TCU.GearChange = 0;
}

static void gear_change_3_4() {
	TCU.GearChange = 1;

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_sln(get_sln_pressure());

	TCU.GearChangeTPS = TCU.InstTPS;
	TCU.GearChangeSLT = TCU.SLT;
	TCU.GearChangeSLN = TCU.SLN;
	
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
	gear_change_wait(GearChangeStep * 20, 5);

	TCU.Gear = 5;
	TCU.GearChange = 0;	
}

//=========================== Переключения вниз ===============================
static void gear_change_5_4() {
	TCU.GearChange = -1;

	// Подгазовка.
	if (TCU.InstTPS < TPS_IDLE_LIMIT) {SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);}
	
	if (TCU.Glock) {			// Отключаем блокировку ГТ.
		set_slu(SLU_MIN_VALUE);
		TCU.Glock = 64;			// Сброс счётчика блокировки
	}
	set_sln(get_sln_pressure());

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	loop_wait(GearChangeStep * 14);
	set_sln(SLN_IDLE_PRESSURE);
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

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
	loop_wait(GearChangeStep * 10);
	set_sln(SLN_IDLE_PRESSURE);

	TCU.Gear = 3;
	TCU.GearChange = 0;	
}

static void gear_change_3_2() {
	TCU.GearChange = -1;

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);	// Включаем систему "Clutch to Clutch".
	SET_PIN_LOW(SOLENOID_S4_PIN);

	if (TCU.TPS >= TPS_IDLE_LIMIT) {
		TCU.Gear2State = 1;
	}
	else {
		set_slu(SLU_MIN_VALUE);
		TCU.Gear2State = 0;
	}

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

	set_slu(SLU_MIN_VALUE);
	set_sln(get_sln_pressure());
	loop_wait(GearChangeStep * 10);
	set_sln(SLN_IDLE_PRESSURE);

	TCU.Gear = 1;
	TCU.Gear2State = 0;
	TCU.GearChange = 0;
}

//========================== Вспомогательные функции ==========================

static void gear_change_wait(uint16_t Delay, uint8_t Gear) {
	uint8_t PDR = 0;
	uint16_t PDRTime = 0;	// Для фиксации времени работы PDR.
	if (TCU.InstTPS > PDR_MAX_TPS) {PDR = -1;}
	TCU.LastPDRTime = 0;
	WaitTimer = Delay;		// Устанавливаем время ожидания.

	while (WaitTimer && rpm_delta(Gear) > 30) {
		set_sln(get_sln_pressure());
		if (rpm_delta(Gear - 1) < -100) {	// Переключение началось.
			if (TCU.Glock) {				// Отключаем блокировку ГТ.
				set_slu(SLU_MIN_VALUE);
				TCU.Glock = 64;				// Сброс счётчика блокировки
			}
			if (!PDR) {						// Запрашиваем снижение мощности.
				PDR = 1;
				PDRTime = WaitTimer;
				SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);
			}
		}
		loop_main(1);
	}

	if (PDR == 1) {
		TCU.LastPDRTime = PDRTime - WaitTimer + 200;
		loop_wait(200);
	}
	set_sln(SLN_IDLE_PRESSURE);
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);
}

void slu_gear2_control(uint8_t Time) {
	static uint16_t Timer = 0;	// Таймер подсчета шагов.

	// 0 - ХХ,
	// 1 - начало включения по оборотам,
	// 2 - продолжение включения по таймеру,
	// 8 - рабочий режим.

	if (TCU.Gear != 2) {
		TCU.GearStep = 0;
		return;
	}

	int16_t RPMDelta = rpm_delta(2);
	int16_t SLUAdd = 0;
	uint16_t NextSLU = get_slu_pressure_gear2();

	switch (TCU.Gear2State) {
		case 0:
			TCU.GearStep = 0;
			SET_PIN_HIGH(SOLENOID_S3_PIN);
			// В режимах "2" и "3", должно быть торможение двигателем.
			if (TCU.ATMode == 6 || TCU.ATMode == 7) {
				set_slu(NextSLU);
				TCU.Gear2State = 2;
			}
			else {
				if (TCU.TPS < TPS_IDLE_LIMIT) {
					TCU.Gear2State = 0;
					set_slu(SLU_MIN_VALUE);
				}
				else {
					if (RPMDelta > -200) {
						set_slu(NextSLU);
						TCU.GearStep = 0;
						TCU.Gear2State = 2;
					}
					else {TCU.Gear2State = 1;}
				} 
			}
			break;
		case 1:
			if (RPMDelta <= -50) {
				if (RPMDelta <= -500) {
					set_slu(SLU_MIN_VALUE);
					TCU.Gear2State = 0;
				}
				else {
					SLUAdd = get_slu_add_gear2() + RPMDelta / 16;
					if (SLUAdd > 0)	{TCU.GearStep = SLUAdd / 2;}
					else {TCU.GearStep = 0;}
					NextSLU += SLUAdd;
					set_slu(NextSLU);
				}
			}
			else {
				SLUAdd = get_slu_add_gear2() + RPMDelta / 16;
				if (SLUAdd > 0)	{TCU.GearStep = SLUAdd / 2;}
				else {TCU.GearStep = 0;}
				NextSLU += SLUAdd;
				set_slu(NextSLU);
				TCU.Gear2State = 2;
			}
			break;
		case 2:
			// Плавное включение второй передачи.
			if (TCU.GearStep < 20) {
				Timer += Time;
				if (Timer >= GearChangeStep) {
					Timer -= GearChangeStep;
					TCU.GearStep++;
				}
				if (TCU.GearStep == 20) {
					SET_PIN_LOW(SOLENOID_S3_PIN);
					TCU.Gear2State = 8;
				}
			}
			NextSLU += TCU.GearStep * 2;
			// Прирост не более чем 8 единиц за цикл.
			if (NextSLU > TCU.SLU) {
				if (NextSLU - TCU.SLU <= 8) {set_slu(NextSLU);}
				else {set_slu(TCU.SLU + 8);}
			}
			else {set_slu(NextSLU);}
			break;
		case 8:
			if (TCU.TPS < TPS_IDLE_LIMIT && TCU.ATMode != 6 && TCU.ATMode != 7) {
				TCU.Gear2State = 0;
				set_slu(SLU_MIN_VALUE);
			}
			else {
				NextSLU = (uint16_t) NextSLU + ((NextSLU * 32) >> 7);	// +25%
				set_slu(NextSLU);
			}
			break;
	}
}

// Первоначальная накачка давления SLU для включения второй передачи.
static void slu_boost() {
	// Ограничение по ДПДЗ и температуре масла.
	if (TCU.InstTPS > 35) {return;}

	set_slu(500);
	loop_wait(GearChangeStep * 2);
	set_slu(get_slu_pressure_gear2());
}

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
void gear_control() {
	// Только режимы D - L.
	if (TCU.ATMode < 4 || TCU.ATMode > 8) {return;}
	if (TCU.Gear < 1 || TCU.Gear > 5) {return;}

	set_gear_change_delays();	// Установка времени на переключение от ДПДЗ.

	TCU.GearUpSpeed = get_gear_max_speed(TCU.Gear);		// Верхняя граница переключения.
	TCU.GearDownSpeed = get_gear_min_speed(TCU.Gear);	// Нижняя граница переключения.

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
	if (TCU.CarSpeed > TCU.GearUpSpeed) {
		if (TCU.InstTPS > 2) {					// Не повышать передачу при сбросе газа.
			if (rpm_after_ok(1)) {gear_up();}
		}
		return;
	}
	// Скорость ниже порога.
	if (TCU.CarSpeed < TCU.GearDownSpeed) {
		if (rpm_after_ok(-1)) {gear_down();}
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
	GearChangeStep = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, GearChangeStepArray, TPS_GRID_SIZE) / 16;
}

// Ожидание с основным циклом.
static void loop_wait(uint16_t Delay) {
	WaitTimer = Delay;		// Устанавливаем время ожидания.
	while (WaitTimer) {loop_main(1);}
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
	return get_interpolated_value_uint16_t(TCU.TPS, TPSGrid, Array, TPS_GRID_SIZE) / 16;
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
	return get_interpolated_value_uint16_t(TCU.TPS, TPSGrid, Array, TPS_GRID_SIZE) / 16;
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

static void set_sln(uint16_t Value) {
	TCU.SLN = Value;
	cli();
		OCR1B = TCU.SLN;	// SLN - выход B таймера 1.
	sei();
}

static void set_slu(uint16_t Value) {
	TCU.SLU = Value;
	cli();
		OCR1C = TCU.SLU;	// SLU - выход C таймера 1.
	sei();
}
