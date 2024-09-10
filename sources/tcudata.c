#include <stdint.h>				// Коротние название int.
#include <avr/io.h>				// Названия регистров и номера бит.

#include "tcudata_tables.h"		// Таблицы TCUData.
#include "tcudata.h"			// Свой заголовок.
#include "spdsens.h"			// Датчики скорости валов.
#include "adc.h"				// АЦП.
#include "macros.h"				// Макросы.
#include "configuration.h"		// Настройки.
#include "mathemat.h"			// Математические функции.
#include "pinout.h"				// Список назначенных выводов.

// Прототипы локальных функций.
static uint16_t get_car_speed();
static uint16_t get_speed_timer_value();

// Инициализация структуры
TCU_t TCU = {
	.DrumRPM = 0,
	.OutputRPM = 0,
	.CarSpeed = 0,
	.SpdTimerVal = 0,
	.OilTemp = 0,
	.TPS = 0,
	.InstTPS = 0,
	.SLT = 0,
	.SLN = 0,
	.SLU = 0,
	.S1 = 0,
	.S2 = 0,
	.S3 = 0,
	.S4 = 0,
	.Selector = 0,
	.ATMode = 0,
	.Gear = 0,
	.GearChange = 0,
	.Break = 0,
	.EngineWork = 0,
	.SlipDetected = 0,
	.Glock = 0,
	.GearUpSpeed = 0,
	.GearDownSpeed = 0
};

// Расчет параметров на основе датчиков и таблиц.
void calculate_tcu_data() {
	TCU.DrumRPM = get_overdrive_drum_rpm();
	TCU.OutputRPM = get_output_shaft_rpm();
	TCU.CarSpeed = get_car_speed();
	TCU.SpdTimerVal = get_speed_timer_value();
	
	TCU.OilTemp = get_oil_temp();

	TCU.S1 = PIN_READ(SOLENOID_S1_PIN) ? 1 : 0;
	TCU.S2 = PIN_READ(SOLENOID_S2_PIN) ? 1 : 0;
	TCU.S3 = PIN_READ(SOLENOID_S3_PIN) ? 1 : 0;
	TCU.S4 = PIN_READ(SOLENOID_S4_PIN) ? 1 : 0;

	slip_detect();
}

// Расчет скорости авто.
static uint16_t get_car_speed() {
	// Расчет скорости автомобиля происходит по выходному валу АКПП.
	// Главная пара - 3.909,
	// Длина окружности колеса - 1.807 м,
	// Коэффициент для оборотов = 1 / 3.909 * 1.807 * 60 / 1000 = 0.027735994
	// Умножаем на 4096 (смещение 12 бит) = 113.6066309
	return ((uint32_t) TCU.OutputRPM * 114) >> 12;
}

// Расчет значения регистра сравнения для таймера спидометра.
static uint16_t get_speed_timer_value() {
	// Таймер 3, делитель х64, Частота 250 кГц, 1 шаг таймера 4 мкс.
	// Количество импульсов на 1 км для спидометра - 16000.
	// [Частота для спидометра] = (16000 / 3600) * [Скорость].
	// [Значение для счетчика OCR3A] = (125000 * 3600) / (16000  * [Скорость])
	// 125000 - это 1000000 мкс в 1с разделить на шаг таймера и еще на 2.
	// Рассчитываем итоговый коэффициент для вычисления (28125).
	#define SPEED_FREQ_COEF 125000LU * 3600LU / SPEED_INPULS_PER_KM
	return SPEED_FREQ_COEF / TCU.CarSpeed;
}

// Расчет температуры масла.
int16_t get_oil_temp() {
	// Датчик температуры находтся на ADC0.
	int16_t TempValue = get_adc_value(0);
	return get_interpolated_value_int16_t(TempValue, OilTempGraph, TempGrid, TEMP_GRID_SIZE);
}

// Положение дросселя.
void calc_tps() {
	// ДПДЗ на ADC1.
	int16_t TempValue = get_adc_value(1);
	TCU.InstTPS = get_interpolated_value_int16_t(TempValue, TPSGraph, TPSGrid, TPS_GRID_SIZE);

	if (TCU.InstTPS >= TCU.TPS) {TCU.TPS = TCU.InstTPS;}
	else {TCU.TPS -= 1;}
}

uint8_t get_slt_pressure() {
	// Управление соленоида SLT инвертирование,
	// но инверсия уже реализована на уровне таймера ШИМ.
	// Потому здесь все линейно, больше значение -> больше давление.

	// Вычисляем значение в зависимости от ДПДЗ.
	uint8_t SLT = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLTGraph, TPS_GRID_SIZE);

	// Применяем коррекцию по температуре.
	SLT = CONSTRAIN(SLT + get_slt_temp_corr(), 0, 255);

	// Добавка давления SLT в режиме "R" и "1". +10%.
	if (TCU.Gear == -1 || TCU.Gear == 1) {SLT = CONSTRAIN(SLT + SLT_ADD_R1, 0, 255);}
	return SLT;
}

int8_t get_slt_temp_corr() {
	int8_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLTTempCorrGraph, TEMP_GRID_SIZE);
	return OilTempCorr;
}

uint8_t get_sln_pressure() {
	// Вычисляем значение в зависимости от ДПДЗ.
	uint8_t SLN = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLNGraph, TPS_GRID_SIZE);
	return SLN;
}

// Давление включения и работы второй передачи SLU B3.
uint8_t get_slu_pressure_gear2() {
	uint8_t PressureGear2 = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLUGear2Graph, TPS_GRID_SIZE);
	// Применяем коррекцию по температуре.
	PressureGear2 = CONSTRAIN(PressureGear2 + get_slu_gear2_temp_corr(), 0, 255);
	return PressureGear2;
}
	
int8_t get_slu_gear2_temp_corr() {
	int8_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLUGear2TempCorrGraph, TEMP_GRID_SIZE);
	return OilTempCorr;
}

// Давление включения третьей передачи SLU B2.
uint8_t get_slu_pressure_gear3() {
	uint8_t PressureGear3 = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLUGear3Graph, TPS_GRID_SIZE);
	return PressureGear3;
}

void slip_detect() {
	if (TCU.GearChange) {		// Не проверять при смене передачи.
		TCU.SlipDetected = 0;
		return;
	}

	if (TCU.InstTPS < 6) {		// Не проверять на малом газу.
		TCU.SlipDetected = 0;
		return;		
	}

	// Расчетная скорость входного вала.
	uint16_t CalcDrumRPM = 0;

	switch (TCU.Gear) {
		case 1:
			CalcDrumRPM = ((uint32_t) TCU.OutputRPM * GEAR_1_RATIO) >> 10;
			break;
		case 2:
			CalcDrumRPM = ((uint32_t) TCU.OutputRPM * GEAR_2_RATIO) >> 10;
			break;
		case 3:
			CalcDrumRPM = ((uint32_t) TCU.OutputRPM * GEAR_3_RATIO) >> 10;
			break;
		case 4:
			CalcDrumRPM = ((uint32_t) TCU.OutputRPM * GEAR_4_RATIO) >> 10;
			break;
		default:
			TCU.SlipDetected = 0;
			return;
	}

	if (TCU.DrumRPM > CalcDrumRPM && TCU.DrumRPM - CalcDrumRPM > MAX_SLIP_RPM) {
		TCU.SlipDetected = 1;
	}
	else {
		TCU.SlipDetected = 0;
	}
}

