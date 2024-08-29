#include <stdint.h>			// Коротние название int.
#include <avr/io.h>			// Названия регистров и номера бит.

#include "gears.h"			// Свой заголовок.
#include "gears_tables.h"	// Таблицы скоростей переключения передач.
#include "pinout.h"			// Список назначенных выводов.
#include "macros.h"			// Макросы.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "configuration.h"	// Настройки.


#define DELAY_AFTER_SHIFT 500
#define SLN_MIN_VALUE 25

// Таймер ожидания.
int16_t WaitTimer = 0;

// Максимальная и минимальная передача для каждого режима.
//						  I  P   R  N  D  D4 D3 L2 L  E  M
//						  0  1   2  3  4  5  6  7  8  9  10
const int8_t MaxGear[] = {0, 0, -1, 0, 5, 4, 3, 2, 1, 0, 5};
const int8_t MinGear[] = {0, 0, -1, 0, 1, 1, 1, 2, 1, 0, 1};

// Прототипы функций.
void loop_main();	// Прототип функций из main.c.

static void gear_up();
static void gear_down();
static void loop_wait(int16_t Delay);
static uint8_t rpm_after_ok(uint8_t Shift);
static void set_slt(uint8_t Value);
static void set_sln(uint8_t Value);
static void set_slu(uint8_t Value);
static uint8_t get_interpolated_value_uint8_t(uint16_t x, uint8_t* ArrayY, uint8_t ArraySize);
static uint16_t get_interpolated_value_uint16_t(uint16_t x, uint16_t* ArrayY, uint8_t ArraySize);

static void gear_change_1_2();
static void gear_change_2_3();
static void gear_change_3_4();
static void gear_change_4_5();

static void gear_change_5_4();
static void gear_change_4_3();
static void gear_change_3_2();
static void gear_change_2_1();

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

// Контроль переключения передач.
void gear_control() {
	// Только режимы D - L.
	if (TCU.ATMode < 4 || TCU.ATMode > 8) {return;}

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
	if (TCU.CarSpeed > get_gear_max_speed()) {
		gear_up();
		return;
	}
	// Скорость ниже порога.
	if (TCU.CarSpeed < get_gear_min_speed()) {
		gear_down();
		return;
	}
}

// Переключение вверх.
static void gear_up() {
	if (TCU.Gear >= MaxGear[TCU.ATMode]) {return;}

	uint8_t ArraySize = sizeof(GearChangeTime) / sizeof(GearChangeTime[0]);
	uint16_t ChangeTime = get_interpolated_value_uint16_t(TCU.TPS, GearChangeTime, ArraySize);

	switch (TCU.Gear) {
		case 1:
			gear_change_1_2(ChangeTime);
			break;
		case 2:
			gear_change_2_3(ChangeTime);
			break;
		case 3:
			gear_change_3_4(ChangeTime);
			break;
		case 4:
			gear_change_4_5(ChangeTime);
			break;
	}
}

// Переключение вниз.
static void gear_down() {
	if (TCU.Gear <= MinGear[TCU.ATMode]) {return;}

	uint8_t ArraySize = sizeof(GearChangeTime) / sizeof(GearChangeTime[0]);
	uint16_t ChangeTime = get_interpolated_value_uint16_t(TCU.TPS, GearChangeTime, ArraySize);

	switch (TCU.Gear) {
		case 2:
			gear_change_2_1(ChangeTime);
			break;
		case 3:
			gear_change_3_2(ChangeTime);
			break;
		case 4:
			gear_change_4_3(ChangeTime);
			break;
		case 5:
			gear_change_5_4(ChangeTime);
			break;
	}	
}

//============================ Начальные передачи =============================
// Включение нейтрали.
void set_gear_n(int16_t Delay) {
	TCU.GearChange = 1;
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_sln(get_sln_value()); 		// Соленоид SLN.

	if (Delay) {loop_wait(Delay);}	// Пауза после включения нейтрали.
	TCU.Gear = 0;
	TCU.GearChange = 0;
}

// Включение первой передачи.
void set_gear_1(int16_t Delay) {
	TCU.GearChange = 1;

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	loop_wait(500);			// Ждем 500 мс.
	set_slt(255);			// Соленоид SLT 100% (дожимаем).
	set_sln(SLN_MIN_VALUE); // Соленоид SLN минимум.

	if (Delay) {loop_wait(Delay);}	// Пауза после включения передачи.
	TCU.Gear = 1;
	TCU.GearChange = 0;
}

// Включение задней передачи.
void set_gear_r(int16_t Delay) {
	TCU.GearChange = 1;
	
	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	loop_wait(500);				// Ждем 500 мс.
	set_slt(255);				// Соленоид SLT 100% (дожимаем).
	set_sln(SLN_MIN_VALUE); 	// Соленоид SLN минимум.

	if (Delay) {loop_wait(Delay);}	// Пауза после включения передачи.
	TCU.Gear = -1;
	TCU.GearChange = 0;
}

//=========================== Переключения вверх ==============================
static void gear_change_1_2(int16_t Delay) {
	TCU.GearChange = 1;

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);	// Переключение SLU на B3.
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_slu(100);	// Порог схватывания фрикциона.
	loop_wait(Delay / 2);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.
	// Плавно добавляем значение.
	for (uint8_t i = 0; i < 8; i++) {
		set_slu(MIN(TCU.SLU + 20, 255));
		loop_wait(Delay / 10);
	}

	set_slt(255);					// Соленоид SLT 100% (дожимаем).
	SET_PIN_LOW(SOLENOID_S3_PIN);	// Отключение SLU.
	set_slu(0);

	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	if (Delay) {loop_wait(DELAY_AFTER_SHIFT);}	// Пауза после включения передачи.
	TCU.Gear = 2;
	TCU.GearChange = 0;
}

static void gear_change_2_3(int16_t Delay) {
	TCU.GearChange = 1;

	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(Delay / 2);

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима 3. 
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.

	loop_wait(Delay);			// Ждем 500 мс.
	set_slt(255);				// Соленоид SLT 100% (дожимаем).
	set_sln(SLN_MIN_VALUE); 	// Соленоид SLN минимум.

	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	if (Delay) {loop_wait(DELAY_AFTER_SHIFT);}	// Пауза после включения передачи.
	TCU.Gear = 3;
	TCU.GearChange = 0;
}

static void gear_change_3_4(int16_t Delay) {
	TCU.GearChange = 1;

	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(Delay / 2);

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.

	loop_wait(Delay);		// Ждем 500 мс.
	set_slt(255);		// Соленоид SLT 100% (дожимаем).
	set_sln(SLN_MIN_VALUE); // Соленоид SLN минимум.

	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	if (Delay) {loop_wait(DELAY_AFTER_SHIFT);}	// Пауза после включения передачи.
	TCU.Gear = 4;
	TCU.GearChange = 0;	
}

static void gear_change_4_5(int16_t Delay) {
	TCU.GearChange = 1;

	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(Delay / 2);

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_HIGH(SOLENOID_S4_PIN);

	SET_PIN_HIGH(REQUEST_POWER_DOWN_PIN);	// Запрос снижения мощности.

	loop_wait(Delay);			// Ждем 500 мс.
	set_slt(255);				// Соленоид SLT 100% (дожимаем).
	set_sln(SLN_MIN_VALUE); 	// Соленоид SLN минимум.

	SET_PIN_LOW(REQUEST_POWER_DOWN_PIN);

	if (Delay) {loop_wait(DELAY_AFTER_SHIFT);}	// Пауза после включения передачи.
	TCU.Gear = 5;
	TCU.GearChange = 0;	
}

//=========================== Переключения вниз ===============================
static void gear_change_5_4(int16_t Delay) {
	TCU.GearChange = 1;

	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(Delay / 2);

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);

	loop_wait(Delay);			// Ждем 500 мс.
	set_slt(255);				// Соленоид SLT 100% (дожимаем).
	set_sln(SLN_MIN_VALUE); 	// Соленоид SLN минимум.

	if (Delay) {loop_wait(DELAY_AFTER_SHIFT);}	// Пауза после включения передачи.
	TCU.Gear = 4;
	TCU.GearChange = 0;		
}

static void gear_change_4_3(int16_t Delay) {
	TCU.GearChange = 1;

	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(Delay / 2);

	SET_PIN_LOW(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима 3. 
	if (TCU.ATMode == 6) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	loop_wait(Delay);			// Ждем 500 мс.
	set_slt(255);				// Соленоид SLT 100% (дожимаем).
	set_sln(SLN_MIN_VALUE); 	// Соленоид SLN минимум.

	if (Delay) {loop_wait(DELAY_AFTER_SHIFT);}	// Пауза после включения передачи.
	TCU.Gear = 3;
	TCU.GearChange = 0;	
}

static void gear_change_3_2(int16_t Delay) {
	TCU.GearChange = 1;

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_HIGH(SOLENOID_S2_PIN);
	SET_PIN_HIGH(SOLENOID_S3_PIN);	// Переключение SLU на B3.
	SET_PIN_LOW(SOLENOID_S4_PIN);

	set_slu(100);	// Порог схватывания фрикциона.
	loop_wait(Delay / 2);

	// Плавно добавляем значение.
	for (uint8_t i = 0; i < 8; i++) {
		set_slu(MIN(TCU.SLU + 20, 255));
		loop_wait(Delay / 10);
	}

	set_slt(255);					// Соленоид SLT 100% (дожимаем).
	SET_PIN_LOW(SOLENOID_S3_PIN);	// Отключение SLU.
	set_slu(0);

	if (Delay) {loop_wait(DELAY_AFTER_SHIFT);}	// Пауза после включения передачи.
	TCU.Gear = 2;
	TCU.GearChange = 0;	
}

static void gear_change_2_1(int16_t Delay) {
	TCU.GearChange = 1;

	set_sln(get_sln_value()); 		// Соленоид SLN.
	loop_wait(Delay / 2);

	SET_PIN_HIGH(SOLENOID_S1_PIN);
	SET_PIN_LOW(SOLENOID_S2_PIN);
	SET_PIN_LOW(SOLENOID_S3_PIN);
	SET_PIN_LOW(SOLENOID_S4_PIN);
	// Отличие для режима L2. 
	if (TCU.ATMode == 7) {SET_PIN_HIGH(SOLENOID_S3_PIN);}

	loop_wait(Delay);			// Ждем 500 мс.
	set_slt(255);				// Соленоид SLT 100% (дожимаем).
	set_sln(SLN_MIN_VALUE); 	// Соленоид SLN минимум.

	if (Delay) {loop_wait(DELAY_AFTER_SHIFT);}	// Пауза после включения передачи.
	TCU.Gear = 1;
	TCU.GearChange = 0;
}

//========================== Вспомогательные функции ==========================
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

	if (TCU.Gear == 5 || TCU.Gear <= 0) {return 255;}

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
	return get_interpolated_value_uint8_t(TCU.CarSpeed, Array, ArraySize);
}

uint8_t get_gear_min_speed() {
	if (TCU.Gear <= 1) {return 0;}

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
	return get_interpolated_value_uint8_t(TCU.CarSpeed, Array, ArraySize);
}

static uint8_t rpm_after_ok(uint8_t Shift) {
	uint8_t Gear = TCU.Gear + Shift;
	uint16_t NewRPM = 0;

	switch (Gear) {
		case 1:
			NewRPM = (TCU.OutputRPM * GEAR_1_RATIO) >> 10;
			break;
		case 2:
			NewRPM = (TCU.OutputRPM * GEAR_2_RATIO) >> 10;
			break;
		case 3:
			NewRPM = (TCU.OutputRPM * GEAR_3_RATIO) >> 10;
			break;
		case 4:
			NewRPM = (TCU.OutputRPM * GEAR_4_RATIO) >> 10;
			break;
		case 5:
			NewRPM = (TCU.OutputRPM * GEAR_5_RATIO) >> 10;
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

// Возвращаент интерполированное значение uint8_t из графика.
static uint8_t get_interpolated_value_uint8_t(uint16_t x, uint8_t* ArrayY, uint8_t ArraySize) {
	uint16_t Result = 0;
	uint8_t StepX = 10; 

	if (x <= 0) {return ArrayY[0];}
	if (x >= (ArraySize - 1) * StepX) {return ArrayY[ArraySize - 1];}		

	// Находим позицию в графике.
	for (uint8_t i = 0; i < ArraySize; i++) {
		if (x <= i * StepX) {
			// Находим значение с помощью интерполяции.
			uint16_t x0 = (i - 1) * StepX;
			uint16_t x1 = i * StepX;
			
			uint16_t y0 = ArrayY[i - 1];
			uint16_t y1 = ArrayY[i];
			Result = y0 * 16 + (((y1 - y0) * 16) * (x - x0)) / (x1 - x0);
			break;
		}
	}

	Result /= 16;
	return Result;
}


// Возвращаент интерполированное значение uint16_t из графика.
static uint16_t get_interpolated_value_uint16_t(uint16_t x, uint16_t* ArrayY, uint8_t ArraySize) {
	uint16_t Result = 0;
	uint8_t StepX = 10; 

	if (x <= 0) {return ArrayY[0];}
	if (x >= (ArraySize - 1) * StepX) {return ArrayY[ArraySize - 1];}		

	// Находим позицию в графике.
	for (uint8_t i = 0; i < ArraySize; i++) {
		if (x <= i * StepX) {
			// Находим значение с помощью интерполяции.
			uint16_t x0 = (i - 1) * StepX;
			uint16_t x1 = i * StepX;
			
			uint16_t y0 = ArrayY[i - 1];
			uint16_t y1 = ArrayY[i];
			Result = y0 * 16 + (((y1 - y0) * 16) * (x - x0)) / (x1 - x0);
			break;
		}
	}

	Result /= 16;
	return Result;
}