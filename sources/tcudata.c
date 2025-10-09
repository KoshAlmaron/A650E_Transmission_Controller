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

// Инициализация структуры
TCU_t TCU = {
	.EngineRPM = 0,
	.DrumRPM = 0,
	.OutputRPM = 0,
	.CarSpeed = 0,
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
	.GearStep = 0,
	.LastStep = 0,
	.Gear2State = 0,
	.Break = 0,
	.EngineWork = 0,
	.SlipDetected = 0,
	.Glock = 0,
	.GearUpSpeed = 0,
	.GearDownSpeed = 0,
	.GearChangeTPS = 0,
	.GearChangeSLT = 0,
	.GearChangeSLN = 0,
	.GearChangeSLU = 0,
	.LastPDRTime = 0,
	.CycleTime = 0,
	.DebugMode = 0
};

 uint8_t SpeedTestFlag = 0;	// Флаг включения тестирования скорости.

// Прототипы локальных функций.
static uint16_t get_car_speed();

// Расчет параметров на основе датчиков и таблиц.
void calculate_tcu_data() {
	TCU.DrumRPM = get_overdrive_drum_rpm();
	TCU.OutputRPM = get_output_shaft_rpm();
	TCU.CarSpeed = get_car_speed();

	TCU.OilTemp = get_oil_temp();

	TCU.S1 = PIN_READ(SOLENOID_S1_PIN) ? 1 : 0;
	TCU.S2 = PIN_READ(SOLENOID_S2_PIN) ? 1 : 0;
	TCU.S3 = PIN_READ(SOLENOID_S3_PIN) ? 1 : 0;
	TCU.S4 = PIN_READ(SOLENOID_S4_PIN) ? 1 : 0;
}

// Расчет скорости авто.
static uint16_t get_car_speed() {
	// Расчет скорости автомобиля происходит по выходному валу АКПП.
	// Главная пара - 3.909,
	// Длина окружности колеса - 1.807 м,
	// Коэффициент для оборотов = 1 / 3.909 * 1.807 * 60 / 1000 = 0.027735994,
	// Умножаем на 4096 (смещение 12 бит) = 113.6066309,
	// Округляем до целого и получается 114.

	//return TCU.InstTPS;

	if (SpeedTestFlag) {return 100;}
	else {return ((uint32_t) TCU.OutputRPM * 114) >> 12;}
}

// Расчет значения регистра сравнения для таймера спидометра.
uint16_t get_speed_timer_value() {
	// Таймер 3, делитель х64, Частота 250 кГц, 1 шаг таймера 4 мкс.
	// Количество импульсов на 1 км для спидометра - 6000.
	// [Частота для спидометра] = (6000 / 3600) * [Скорость].
	// [Значение для счетчика OCR3A] = (125000 * 3600) / (6000  * [Скорость])
	// 125000 - это 1000000 мкс в 1с разделить на шаг таймера и еще на 2.
	// Рассчитываем итоговый коэффициент для вычисления (28125).

	#define SPEED_FREQ_COEF 125000LU * 3600LU / SPEED_INPULS_PER_KM

	if (TCU.CarSpeed == 0) {return 0;}
	else {return ((uint32_t) SPEED_FREQ_COEF / TCU.CarSpeed);}
}

// Расчет температуры масла.
int16_t get_oil_temp() {
	// Датчик температуры находтся на ADC0.
	int16_t TempValue = get_adc_value(0);
	return get_interpolated_value_int16_t(TempValue, OilTempGraph, TempGrid, TEMP_GRID_SIZE) / 16;
}

// Положение дросселя.
void calc_tps() {
	// ДПДЗ на ADC1.
	static uint8_t Counter = 0;

	int16_t TempValue = get_adc_value(1);
	TCU.InstTPS = get_interpolated_value_int16_t(TempValue, TPSGraph, TPSGrid, TPS_GRID_SIZE) / 16;
	if (TCU.InstTPS >= TCU.TPS) {
		TCU.TPS = TCU.InstTPS;
	}
	else {
		// Плавное снижение ДПДЗ
		Counter++;
		if (Counter > 1) {
			Counter = 0;
			TCU.TPS -= 1;
		}
	}
}

uint16_t get_slt_pressure() {
	// Управление соленоида SLT инвертирование,
	// но инверсия уже реализована на уровне таймера ШИМ.
	// Потому здесь все линейно, больше значение -> больше давление.

	// Вычисляем значение в зависимости от ДПДЗ.
	uint16_t SLT = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLTGraph, TPS_GRID_SIZE) / 16;
	// Применяем коррекцию по температуре.
	SLT = CONSTRAIN(SLT + get_slt_temp_corr(SLT), 80, 980);
	return SLT;
}

// Возращает коррекцию в процентах или сразу рассчитанную добавку,
// если передать функции базовое значение.
int16_t get_slt_temp_corr(int16_t Value) {
	int32_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLTTempCorrGraph, TEMP_GRID_SIZE);
	
	if (!Value) {return (OilTempCorr / 16);}	// Возвращаем коррекцию в %.
	else {	// Возвращаем скорректированное значение.
		OilTempCorr = (Value * OilTempCorr) / 1600;	// Коррекция в значениях ШИМ.
		return OilTempCorr;
	}
}

uint16_t get_sln_pressure() {
	// Вычисляем значение в зависимости от ДПДЗ.
	uint16_t SLN = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLNGraph, TPS_GRID_SIZE) / 16;
	return SLN;
}

uint16_t get_sln_pressure_gear3() {
	// Вычисляем значение в зависимости от ДПДЗ.
	uint16_t SLN = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLNGear3Graph, TPS_GRID_SIZE) / 16;
	return SLN;
}

// Давление включения и работы второй передачи SLU B3.
uint16_t get_slu_pressure_gear2() {
	uint16_t SLU = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLUGear2Graph, TPS_GRID_SIZE) / 16;
	
	#ifdef GEAR_2_SLU_TPS_ADAPTATION
		SLU += get_interpolated_value_int16_t(TCU.InstTPS, TPSGrid, SLUGear2TPSAdaptGraph, TPS_GRID_SIZE) / 16;
	#endif

	// Применяем коррекцию по температуре.
	SLU = CONSTRAIN(SLU + get_slu_gear2_temp_corr(SLU), 100, 980);
	return SLU;
}

// Возращает коррекцию в процентах или сразу рассчитанную добавку,
// если передать функции базовое значение.
int16_t get_slu_gear2_temp_corr(int16_t Value) {
	int32_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLUGear2TempCorrGraph, TEMP_GRID_SIZE);

	#ifdef GEAR_2_SLU_TEMP_ADAPTATION
		OilTempCorr += get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLUGear2TempAdaptGraph, TEMP_GRID_SIZE);
	#endif

	if (!Value) {return (OilTempCorr / 16);}	// Возвращаем коррекцию в %.
	else {	// Возвращаем скорректированное значение.
		OilTempCorr = (Value * OilTempCorr) / 1600;    // Коррекция в значениях ШИМ.
		return OilTempCorr;
	}
}

// Добавка к давлению SLU при повторном включении второй передачи.
uint16_t get_slu_add_gear2() {
	return get_interpolated_value_int16_t(TCU.InstTPS, TPSGrid, SLUGear2AddGraph, TPS_GRID_SIZE) / 16;
}

// Давление включения третьей передачи SLU B2.
uint16_t get_slu_pressure_gear3() {
	uint16_t SLU = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLUGear3Graph, TPS_GRID_SIZE) / 16;
	// Применяем коррекцию по температуре.
	SLU = CONSTRAIN(SLU + get_slu_gear2_temp_corr(SLU), 100, 980);
	return SLU;
}

// Задержка отключения SLU при включении третьей передачи.
uint16_t get_gear3_slu_delay() {
	int16_t Delay = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLUGear3DelayGraph, TPS_GRID_SIZE) / 16;

	#ifdef GEAR_3_SLU_TPS_ADAPTATION
		Delay += get_interpolated_value_int16_t(TCU.InstTPS, TPSGrid, SLUGear3TPSAdaptGraph, TPS_GRID_SIZE) / 16;
	#endif

	// Коррекция задержки от температуры.
	Delay += get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLUG3DelayTempCorrGraph, TEMP_GRID_SIZE) / 16;

	#ifdef GEAR_3_SLU_TEMP_ADAPTATION
		Delay += get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLUGear3TempAdaptGraph, TEMP_GRID_SIZE) / 16;
	#endif

	if (Delay < 0) {return 0;}
	else {return Delay;}
}

// Смещение впемени включения SLN при включении третьей передачи.
int16_t get_gear3_sln_offset() {
	int16_t Offset = get_interpolated_value_int16_t(TCU.InstTPS, TPSGrid, SLNGear3OffsetGraph, TPS_GRID_SIZE) / 16;
	return Offset;
}

uint8_t get_tps_index(uint8_t TPS) {
	if (TPS < 3) {return 0;}

	uint8_t Delta = 255;
	for (uint8_t i = 1; i < TPS_GRID_SIZE; i++) {
		uint8_t Diff = 0;
		if (TPS > TPSGrid[i]) {Diff = TPS - TPSGrid[i];}
		else {Diff = TPSGrid[i] - TPS;}

		if (Diff < Delta) {Delta = Diff;}
		else {return i - 1;}
	}
	return 0;
}

uint8_t get_temp_index(int16_t Temp) {
	if (Temp < -27) {return 0;}

	uint8_t Delta = 255;
	for (uint8_t i = 1; i < TEMP_GRID_SIZE; i++) {
		uint8_t Diff = 0;
		if (Temp > TempGrid[i]) {Diff = Temp - TempGrid[i];}
		else {Diff = TempGrid[i] - Temp;}

		if (Diff < Delta) {Delta = Diff;}
		else {return i - 1;}
	}
	return 0;
}

// Расчет разницы скорости входного вала относительно расчетной 
// по датчику скорости и передаточному числу. 
int16_t rpm_delta(uint8_t Gear) {
	if (Gear == 5) {return TCU.DrumRPM;}
	if (!TCU.OutputRPM || (!TCU.DrumRPM && TCU.Gear != 5))  {return 0;}

	// Расчетная скорость входного вала.
	int16_t CalcDrumRPM = 0;
	switch (Gear) {
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
		case 5:	// На пятой передаче барабан овердрайва останавливается.
			return TCU.DrumRPM;
			break;
	}
	return (TCU.DrumRPM - CalcDrumRPM);
}

void save_gear2_adaptation(int8_t Value) {
	// Адаптация по ДПДЗ.
	if (TCU.OilTemp >= GEAR_2_SLU_ADAPTATION_OIL_MIN && TCU.OilTemp <= GEAR_2_SLU_ADAPTATION_OIL_MAX) {
		#ifdef GEAR_2_SLU_TPS_ADAPTATION
			uint8_t Index = 0;
			Index = get_tps_index(TCU.InstTPS);
			SLUGear2TPSAdaptGraph[Index] += (Value * 4);
			SLUGear2TPSAdaptGraph[Index] = CONSTRAIN(SLUGear2TPSAdaptGraph[Index], -32, 32);
		#endif
	}
	else {		// Адаптация по температуре масла.
		#ifdef GEAR_2_SLU_TEMP_ADAPTATION
			if (TCU.InstTPS > GEAR_2_SLU_ADAPTATION_OIL_MAX_TPS) {return;} // Адаптация по температуре только на малом газу.
			uint8_t Index = 0;
			Index = get_temp_index(TCU.OilTemp);
			SLUGear2TempAdaptGraph[Index] += Value;
			SLUGear2TempAdaptGraph[Index] = CONSTRAIN(SLUGear2TempAdaptGraph[Index], -12, 12);
		#endif
	}
}

void save_gear3_adaptation(int8_t Value) {
	// Адаптация по ДПДЗ.
	if (TCU.OilTemp >= GEAR_3_SLU_ADAPTATION_OIL_MIN && TCU.OilTemp <= GEAR_3_SLU_ADAPTATION_OIL_MAX) {
		#ifdef GEAR_3_SLU_TPS_ADAPTATION
			uint8_t Index = 0;
			Index = get_tps_index(TCU.InstTPS);
			SLUGear3TPSAdaptGraph[Index] += (Value * 20);
			SLUGear3TPSAdaptGraph[Index] = CONSTRAIN(SLUGear3TPSAdaptGraph[Index], -200, 200);
		#endif
	}
	else {		// Адаптация по температуре масла.
		#ifdef GEAR_3_SLU_TEMP_ADAPTATION
			if (TCU.InstTPS > GEAR_3_SLU_ADAPTATION_OIL_MAX_TPS) {return;} // Адаптация по температуре только на малом газу.
			uint8_t Index = 0;
			Index = get_temp_index(TCU.OilTemp);
			SLUGear3TempAdaptGraph[Index] += Value * 20;
			SLUGear3TempAdaptGraph[Index] = CONSTRAIN(SLUGear3TempAdaptGraph[Index], -200, 200);
		#endif
	}
}