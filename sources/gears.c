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

#define GEAR_2_MAX_STEP 20			// Количество шагов при включении второй передачи.

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

static void set_solenoids(int8_t Gear);

//============================ Начальные передачи =============================
// Включение нейтрали.
void set_gear_n() {
	TCU.GearChange = -1;
	if (TCU.Gear < 0) {TCU.GearChange = 1;}

	set_slu(SLU_MIN_VALUE);			// Выключение SLU на случай переключения со второй передачи.
	set_sln(get_sln_pressure());
	set_solenoids(0);				// Установка шифтовых соленоидов.

	TCU.Gear = 0;
	TCU.GearChange = 0;
}

// Включение первой передачи.
void set_gear_1() {
	TCU.GearChange = 1;

	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.
	set_sln(get_sln_pressure());
	set_solenoids(1);			// Установка шифтовых соленоидов.
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}	// Отличие для режима L2. 

	TCU.Gear = 1;
	TCU.GearChange = 0;
}

// Включение задней передачи.
void set_gear_r() {
	TCU.GearChange = -1;

	set_slu(SLU_MIN_VALUE);		// Выключение SLU на случай переключения со второй передачи.
	set_sln(get_sln_pressure());
	set_solenoids(-1);			// Установка шифтовых соленоидов.

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
	TCU.GearChange = 1;
	slu_boost();						// Первоначальная накачка давления SLU.

	set_solenoids(2);					// Установка шифтовых соленоидов.
	SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".

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
	int8_t Adaptation = 0;	// Флаг применения адаптации.
	TCU.LastPDRTime = 0;
	if (TCU.InstTPS > PDR_MAX_TPS) {PDR = -1;}
	
	while (TCU.GearStep < GEAR_2_MAX_STEP) {
		set_gear_change_delays();		// Длительность 1 шага переключения от ДПДЗ.
		WaitTimer = GearChangeStep;		// Устанавливаем время ожидания.
		while (WaitTimer) {
			loop_main(1);

			// Отключение передачи при сбросе газа.
			if (TCU.InstTPS < TPS_IDLE_LIMIT && TCU.ATMode != 6 && TCU.ATMode != 7) {
				set_solenoids(1);
				set_slu(SLU_MIN_VALUE);
				loop_wait(200);
				TCU.Gear = 2;
				TCU.Gear2State = 0;
				TCU.GearChange = 0;
				return;
			}

			NextSLU = get_slu_pressure_gear2() + TCU.GearStep * 2 - SLUDelay;
			set_slu(NextSLU);

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

			if (TCU.LastStep == GEAR_2_MAX_STEP) {TCU.LastStep = TCU.GearStep;}
			if (TCU.GearStep < 13 && !Adaptation) {
				// Передача включилась слишком рано,
				// снижаем давление на 1 единицу.
				Adaptation = -1;
				save_gear2_slu_adaptation(-1);
			}
		}
		if (TCU.GearStep > 15 && rpm_delta(2) > 30 && !Adaptation) {
			// Передача включилась слишком поздно,
			// повышаем давление на 1 единицу.
			Adaptation = 1;
			save_gear2_slu_adaptation(1);
		}
	}

	SET_PIN_LOW(SOLENOID_S3_PIN);
	set_sln(SLN_IDLE_PRESSURE);

	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);
	
	TCU.Gear = 2;
	TCU.Gear2State = 8;
	TCU.GearChange = 0;
}

static void gear_change_2_3() {
	// Не включать третью при выключенной второй передаче.
	if (TCU.Gear2State != 8) {return;}
	TCU.GearChange = 1;
	
	set_solenoids(3);		// Установка шифтовых соленоидов.

	set_slu(get_slu_pressure_gear3());
	TCU.GearChangeTPS = TCU.InstTPS;
	TCU.GearChangeSLU = TCU.SLU;

	set_sln(SLN_IDLE_PRESSURE);

	WaitTimer = get_gear3_slu_delay();				// Время удержания давления SLU.
	int16_t SLNOffset = get_gear3_sln_offset();		// Смещение времени включения SLN.
	uint8_t SetSLN = 0;		// Флаг установки давления SLN
	int8_t Adaptation = 0;	// Флаг применения адаптации.

	uint8_t PDR = 0;		// Флаг применения запроса снижения мощности.
	uint16_t PDRTime = 0;	// Длительность применения снижения мощности.

	// Ждем начало включения B2.
	while (WaitTimer) {
		loop_main(1);

		// Давление SLU включения третьей передачи
		set_slu(get_slu_pressure_gear3());
		
		if (!SetSLN) {
			SLNOffset = get_gear3_sln_offset();
		 	if (WaitTimer + SLNOffset < 5) {SetSLN = 1;}	// Пересечение SNL и SLU при SLNOffset < 0.
		}
		else {set_sln(get_sln_pressure_gear3());}
	}
	set_slu(SLU_MIN_VALUE);		// Убираем давление SLU.

	if (TCU.InstTPS > PDR_MAX_TPS) {PDR = -1;}

	WaitTimer = 1500;						// Время ожидания завершения переключения.
	uint16_t GearTestTimer = WaitTimer;		// Таймер для проверки начала переключения.
	// Ждем включения третьей передачи.
	while (WaitTimer && rpm_delta(3) > 30) {
		int16_t Delta2 = rpm_delta(2);

		loop_main(1);

		// Проверка на закусывание передачи 2 и 3.
		// Через 40 мс после сброса давления SLU должно начаться изменение передаточного числа.
		if (GearTestTimer && !Adaptation && GearTestTimer - WaitTimer >= 40) {
			GearTestTimer = 0;
			if (Delta2 > -5) {Adaptation = -1;}	// Произошло закусывание (?).
		}
		
		if (!PDR && Delta2 < -75) {		// Переключение началось.
			PDR = 1;
			PDRTime = WaitTimer;
			SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);
		}

		if (!SetSLN) {
			SLNOffset = get_gear3_sln_offset();
		 	if (1500 - WaitTimer > SLNOffset) {SetSLN = 1;}		// Задержка SLN при SLNOffset > 0.
		}
		else {set_sln(get_sln_pressure_gear3());}				// Устанавливаем давление SLN.

		if (Adaptation != 1 && Delta2 > 40) {
			// Проскальзывание второй передачи.
			// Выключение SLU произошло слишком рано.
			Adaptation = 1;
		}
	}

	TCU.GearChangeSLN = TCU.SLN;
	set_sln(SLN_IDLE_PRESSURE);

	if (PDR == 1) {
		TCU.LastPDRTime = PDRTime - WaitTimer;
		//loop_wait(200);
	}
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	// Включаем торможение двигателем в режиме "3".
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	// Применение адаптации.
	if (Adaptation) {save_gear3_slu_adaptation(Adaptation);}

	TCU.Gear = 3;
	TCU.Gear2State = 0;
	TCU.GearChange = 0;
}

static void gear_change_3_4() {
	TCU.GearChange = 1;

	set_solenoids(4);		// Установка шифтовых соленоидов.

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

	set_solenoids(5);		// Установка шифтовых соленоидов.

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

	set_solenoids(4);		// Установка шифтовых соленоидов.

	loop_wait(GearChangeStep * 14);
	set_sln(SLN_IDLE_PRESSURE);
	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	TCU.Gear = 4;
	TCU.GearChange = 0;		
}

static void gear_change_4_3() {
	TCU.GearChange = -1;
	
	set_solenoids(3);		// Установка шифтовых соленоидов.
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

	if (TCU.ATMode == 6 || TCU.ATMode == 7) {
		// Включение второй передачи в режимах "3" и "L2".
		set_solenoids(2);					// Установка шифтовых соленоидов.
		SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".

		uint16_t NextSLU = get_slu_pressure_gear2();
		set_slu(NextSLU);
		set_sln(SLN_MIN_PRESSURE);

		TCU.GearStep = 0;		// Шаг процесса включения передачи.

		while (TCU.GearStep < GEAR_2_MAX_STEP) {
			set_gear_change_delays();		// Длительность 1 шага переключения от ДПДЗ.
			WaitTimer = GearChangeStep;		// Устанавливаем время ожидания.
			while (WaitTimer) {
				loop_main(1);

				NextSLU = get_slu_pressure_gear2() + TCU.GearStep * 2;
				set_slu(NextSLU);
			}
			TCU.GearStep++;
		}
		SET_PIN_LOW(SOLENOID_S3_PIN);
		set_sln(SLN_IDLE_PRESSURE);
		TCU.Gear2State = 8;
	}
	else {
		// Без торможения двигателем вторая передача будет включаться
		// в функции slu_gear2_control.
		set_solenoids(1);					// Установка шифтовых соленоидов.
		set_slu(SLU_MIN_VALUE);
		loop_wait(100);
		TCU.Gear2State = 0;
	}

	TCU.Gear = 2;
	TCU.GearChange = 0;	
}	

static void gear_change_2_1() {
	TCU.GearChange = -1;

	set_solenoids(1);		// Установка шифтовых соленоидов.
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

void slu_gear2_control() {
	// Дельта оборотов, при котором началось переключение.
	static int16_t InitDrumRPMDelta = 0;
	// 0 - ХХ,
	// 1 - плавное включение,
	// 8 - рабочий режим.

	if (TCU.Gear != 2) {
		TCU.GearStep = 0;
		TCU.Gear2State = 0;
		return;
	}

	// Отключение передачи при сбросе газа.
	if (TCU.InstTPS < TPS_IDLE_LIMIT && TCU.ATMode != 6 && TCU.ATMode != 7) {
		if (TCU.Gear2State > 0) {
			set_solenoids(1);
			set_slu(SLU_MIN_VALUE);
			loop_wait(200);
		}
		TCU.Gear2State = 0;
	}

	int16_t DeltaRPM = rpm_delta(2);
	int16_t MaxDeltaRPM = 0;
	uint16_t NextSLU = get_slu_pressure_gear2();		// Начальное давление.
	uint16_t WorkSLU = NextSLU + ((NextSLU * 32) >> 7);	// Рабочее давление (+25%).

	switch (TCU.Gear2State) {
		case 0:
			// При временном отключении второй передачи, давление SLU не сбрасывается,
			// Вместо этого включается первая передача.
			set_solenoids(1);
			TCU.GearStep = 0;
			set_slu(WorkSLU);
			InitDrumRPMDelta = 0;

			if (TCU.ATMode == 6 || TCU.ATMode == 7) {	// Переключили режим АКПП.
				set_slu(NextSLU);
				loop_wait(200);
				set_solenoids(2);
				SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".
				TCU.Gear2State = 1;
			}
			else if (TCU.InstTPS >= TPS_IDLE_LIMIT && (-1 * DeltaRPM) < get_gear2_rpm_adv()) {
				set_slu(NextSLU);
				loop_wait(200);
				set_solenoids(2);
				SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".
				TCU.Gear2State = 1;
				InitDrumRPMDelta = TCU.DrumRPMDelta;
			}
			break;
		case 1:
			// Плавное включение второй передачи.
			set_solenoids(2);					// Установка шифтовых соленоидов.
			SET_PIN_HIGH(SOLENOID_S3_PIN);		// Включаем систему "Clutch to Clutch".
			TCU.GearStep = 0;
			while (TCU.GearStep < GEAR_2_MAX_STEP) {
				set_gear_change_delays();		// Длительность 1 шага переключения от ДПДЗ.
				// Устанавливаем время ожидания.
				if (TCU.ATMode == 6 || TCU.ATMode == 7) {WaitTimer = GearChangeStep;}
				else {WaitTimer = (GearChangeStep * 3) / 4;}

				while (WaitTimer) {
					loop_main(1);
					DeltaRPM = rpm_delta(2);

					// Отключение передачи при сбросе газа.
					if (TCU.InstTPS < TPS_IDLE_LIMIT && TCU.ATMode != 6 && TCU.ATMode != 7) {
						set_solenoids(1);
						set_slu(SLU_MIN_VALUE);
						loop_wait(200);
						TCU.Gear2State = 0;
						return;
					}

					// Фиксация максимальной разница оборотов (проскальзывание).
					if (DeltaRPM > MaxDeltaRPM) {MaxDeltaRPM = DeltaRPM;}

					NextSLU = get_slu_pressure_gear2() + TCU.GearStep * 2;
					set_slu(NextSLU);
				}
				TCU.GearStep++;
			}
			SET_PIN_LOW(SOLENOID_S3_PIN);
			set_sln(SLN_IDLE_PRESSURE);
			TCU.Gear2State = 8;

			// Применение адаптации.
			if (TCU.ATMode != 6 && TCU.ATMode != 7 && InitDrumRPMDelta) {
				// Обороты проскачили нулевую точку, передача включилась поздно.
				if (MaxDeltaRPM > 80) {save_gear2_adv_adaptation(1, InitDrumRPMDelta);}
				// Обороты сликом близко к нулевойю точку, передача включилась рано.
				else if (MaxDeltaRPM < 40) {save_gear2_adv_adaptation(-1, InitDrumRPMDelta);}
			}
			break;
		case 8:
			set_slu(WorkSLU);
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

	set_gear_change_delays();	// Длительность 1 шага переключения от ДПДЗ.

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
	GearChangeStep = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, GearChangeStepArray, TPS_GRID_SIZE);
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

	uint16_t Speed = get_interpolated_value_uint16_t(TCU.TPS, TPSGrid, Array, TPS_GRID_SIZE);
	return Speed;
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
	uint16_t Speed = get_interpolated_value_uint16_t(TCU.TPS, TPSGrid, Array, TPS_GRID_SIZE);
	return Speed;
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

int8_t get_min_gear(uint8_t Mode) {
	return MinGear[Mode];
}
int8_t get_max_gear(uint8_t Mode) {
	return MaxGear[Mode];
}

void set_gear_limit(uint8_t Min, uint8_t Max) {
	#ifdef SELECTOR_HAS_D4_MODE
		MinGear[5] = Min;
		MaxGear[5] = Max;
	#else
		MinGear[4] = Min;
		MaxGear[4] = Max;
	#endif
}

static void set_solenoids(int8_t Gear) {
	switch (Gear) {
		case -1:
			SET_PIN_HIGH(SOLENOID_S1_PIN);
			SET_PIN_LOW(SOLENOID_S2_PIN);
			SET_PIN_LOW(SOLENOID_S3_PIN);
			SET_PIN_LOW(SOLENOID_S4_PIN);
			break;
		case 0:
			SET_PIN_HIGH(SOLENOID_S1_PIN);
			SET_PIN_LOW(SOLENOID_S2_PIN);
			SET_PIN_HIGH(SOLENOID_S3_PIN);
			SET_PIN_LOW(SOLENOID_S4_PIN);
			break;
		case 1:
			SET_PIN_HIGH(SOLENOID_S1_PIN);
			SET_PIN_LOW(SOLENOID_S2_PIN);
			SET_PIN_LOW(SOLENOID_S3_PIN);
			SET_PIN_LOW(SOLENOID_S4_PIN);
			break;
		case 2:
			SET_PIN_HIGH(SOLENOID_S1_PIN);
			SET_PIN_HIGH(SOLENOID_S2_PIN);
			SET_PIN_LOW(SOLENOID_S3_PIN);
			SET_PIN_LOW(SOLENOID_S4_PIN);
			break;
		case 3:
			SET_PIN_LOW(SOLENOID_S1_PIN);
			SET_PIN_HIGH(SOLENOID_S2_PIN);
			SET_PIN_LOW(SOLENOID_S3_PIN);
			SET_PIN_LOW(SOLENOID_S4_PIN);
			break;
		case 4:
			SET_PIN_LOW(SOLENOID_S1_PIN);
			SET_PIN_LOW(SOLENOID_S2_PIN);
			SET_PIN_HIGH(SOLENOID_S3_PIN);
			SET_PIN_LOW(SOLENOID_S4_PIN);
			break;
		case 5:
			SET_PIN_LOW(SOLENOID_S1_PIN);
			SET_PIN_LOW(SOLENOID_S2_PIN);
			SET_PIN_LOW(SOLENOID_S3_PIN);
			SET_PIN_HIGH(SOLENOID_S4_PIN);
			break;
	}
}