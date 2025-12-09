#include <stdint.h>				// Коротние название int.
#include <avr/io.h>				// Названия регистров и номера бит.

#include "tcudata.h"			// Свой заголовок.
#include "tcudata_tables.h"		// Таблицы TCUData.

#include "spdsens.h"			// Датчики скорости валов.
#include "adc.h"				// АЦП.
#include "macros.h"				// Макросы.
#include "configuration.h"		// Настройки.
#include "mathemat.h"			// Математические функции.
#include "pinout.h"				// Список назначенных выводов.

// Инициализация структуры с переменными.
TCU_t TCU = {
	.EngineRPM = 0,
	.DrumRPM = 0,
	.DrumRPMDelta = 0,
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
	.DebugMode = 0,
	.RawTPS = 0,
	.RawOIL = 0,
	.AdaptationTPS = 0,
	.AdaptationTemp = 0
};

uint8_t SpeedTestFlag = 0;	// Флаг включения тестирования скорости.

// Прототипы локальных функций.
static uint16_t get_car_speed();

static int16_t get_cell_adapt_step(uint8_t N, int16_t Value, int16_t LeftCell, int8_t GridStep, int16_t AdaptStep);

// Расчет параметров на основе датчиков и таблиц.
void calculate_tcu_data() {
	static uint8_t Counter = 0;
	static uint16_t PrevDrumRPM = 0;

	TCU.DrumRPM = get_overdrive_drum_rpm();
	TCU.OutputRPM = get_output_shaft_rpm();
	TCU.CarSpeed = get_car_speed();

	TCU.OilTemp = get_oil_temp();

	TCU.S1 = PIN_READ(SOLENOID_S1_PIN) ? 1 : 0;
	TCU.S2 = PIN_READ(SOLENOID_S2_PIN) ? 1 : 0;
	TCU.S3 = PIN_READ(SOLENOID_S3_PIN) ? 1 : 0;
	TCU.S4 = PIN_READ(SOLENOID_S4_PIN) ? 1 : 0;

	Counter++;
	if (Counter >= 2) {
		Counter = 0;
		TCU.DrumRPMDelta = (TCU.DrumRPM - PrevDrumRPM);
		PrevDrumRPM = TCU.DrumRPM;
	}
}

// Расчет скорости авто.
static uint16_t get_car_speed() {
	// Расчет скорости автомобиля происходит по выходному валу АКПП.
	// Главная пара - 3.909,
	// Длина окружности колеса - 1.807 м,
	// Коэффициент для оборотов = 1 / 3.909 * 1.807 * 60 / 1000 = 0.027735994,
	// Умножаем на 4096 (смещение 12 бит) = 113.6066309,
	// Округляем до целого и получается 114.

	if (SpeedTestFlag) {return 100;}
	else {return ((uint32_t) TCU.OutputRPM * CFG.SpeedCalcCoef) >> 12;}
}

// Расчет значения регистра сравнения для таймера спидометра.
uint16_t get_speed_timer_value() {
	// Таймер 3, делитель х64, Частота 250 кГц, 1 шаг таймера 4 мкс.
	// Количество импульсов на 1 км для спидометра - 6000.
	// [Частота для спидометра] = (6000 / 3600) * [Скорость].
	// [Значение для счетчика OCR3A] = (125000 * 3600) / (6000  * [Скорость])
	// 125000 - это 1000000 мкс в 1с разделить на шаг таймера и еще на 2.
	// Рассчитываем итоговый коэффициент для вычисления (28125).

	#define SPEED_FREQ_COEF 125000LU * 3600LU
	if (TCU.CarSpeed == 0) {return 0;}

	return ((uint32_t) SPEED_FREQ_COEF / ((uint32_t) CFG.SpeedImpulsPerKM * TCU.CarSpeed));
}

// Расчет температуры масла.
int16_t get_oil_temp() {
	// Датчик температуры находтся на ADC0.
	int16_t TempValue = get_adc_value(0);
	TCU.RawOIL = TempValue;
	return get_interpolated_value_int16_t(TempValue, ADCTBL.OilTempGraph, GRIDS.TempGrid, TEMP_GRID_SIZE);
}

// Положение дросселя.
void calc_tps() {
	// ДПДЗ на ADC1.
	static uint8_t Counter = 0;

	int16_t TempValue = get_adc_value(1);
	TCU.RawTPS = TempValue;
	TCU.InstTPS = get_interpolated_value_int16_t(TempValue, ADCTBL.TPSGraph, GRIDS.TPSGrid, TPS_GRID_SIZE);

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
	uint16_t SLT = get_interpolated_value_uint16_t(TCU.InstTPS, GRIDS.TPSGrid, TABLES.SLTGraph, TPS_GRID_SIZE);
	// Применяем коррекцию по температуре.
	SLT = CONSTRAIN(SLT + get_slt_temp_corr(SLT), 80, 980);
	return SLT;
}

// Возращает коррекцию в процентах или сразу рассчитанную добавку,
// если передать функции базовое значение.
int16_t get_slt_temp_corr(int16_t Value) {
	int32_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, GRIDS.TempGrid, TABLES.SLTTempCorrGraph, TEMP_GRID_SIZE);

	if (!Value) {return OilTempCorr;}	// Возвращаем коррекцию в %.
	else {	// Возвращаем скорректированное значение.
		OilTempCorr = (Value * OilTempCorr) / 1024;	// Коррекция в значениях ШИМ.
		return OilTempCorr;
	}
}

uint16_t get_sln_pressure() {
	// Вычисляем значение в зависимости от ДПДЗ.
	uint16_t SLN = get_interpolated_value_uint16_t(TCU.InstTPS, GRIDS.TPSGrid, TABLES.SLNGraph, TPS_GRID_SIZE);
	// Применяем коррекцию по температуре.
	SLN = CONSTRAIN(SLN + get_sln_temp_corr(SLN), 20, 980);
	return SLN;
}

int16_t get_sln_temp_corr(int16_t Value) {
	int32_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, GRIDS.TempGrid, TABLES.SLNTempCorrGraph, TEMP_GRID_SIZE);
	
	if (!Value) {return OilTempCorr;}	// Возвращаем коррекцию в %.
	else {	// Возвращаем скорректированное значение.
		OilTempCorr = (Value * OilTempCorr) / 1024;	// Коррекция в значениях ШИМ.
		return OilTempCorr;
	}
}

uint16_t get_sln_pressure_gear3() {
	// Вычисляем значение в зависимости от ДПДЗ.
	uint16_t SLN = get_interpolated_value_uint16_t(TCU.InstTPS, GRIDS.TPSGrid, TABLES.SLNGear3Graph, TPS_GRID_SIZE);
	return SLN;
}

// Давление включения и работы второй передачи SLU B3.
uint16_t get_slu_pressure_gear2() {
	uint16_t SLU = get_interpolated_value_uint16_t(TCU.InstTPS, GRIDS.TPSGrid, TABLES.SLUGear2Graph, TPS_GRID_SIZE);
	
	if (CFG.G2EnableAdaptTPS) {
		SLU += get_interpolated_value_int16_t(TCU.InstTPS, GRIDS.TPSGrid, ADAPT.SLUGear2TPSAdaptGraph, TPS_GRID_SIZE);
	}

	// Применяем коррекцию по температуре.
	SLU = CONSTRAIN(SLU + get_slu_gear2_temp_corr(SLU), 100, 980);
	return SLU;
}

// Возращает коррекцию в процентах или сразу рассчитанную добавку,
// если передать функции базовое значение.
int16_t get_slu_gear2_temp_corr(int16_t Value) {
	int32_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, GRIDS.TempGrid, TABLES.SLUGear2TempCorrGraph, TEMP_GRID_SIZE);

	if (CFG.G2EnableAdaptTemp) {
		OilTempCorr += get_interpolated_value_int16_t(TCU.OilTemp, GRIDS.TempGrid, ADAPT.SLUGear2TempAdaptGraph, TEMP_GRID_SIZE);
	}

	if (!Value) {return OilTempCorr;}	// Возвращаем коррекцию в %.
	else {	// Возвращаем скорректированное значение.
		OilTempCorr = (Value * OilTempCorr) / 1024;    // Коррекция в значениях ШИМ.
		return OilTempCorr;
	}
}

// Опережение по оборотам реактивации второй передачи.
int16_t get_gear2_rpm_adv() {
	int16_t AdvanceRPM = get_interpolated_value_int16_t(TCU.DrumRPMDelta, GRIDS.DeltaRPMGrid, TABLES.Gear2AdvGraph, DELTA_RPM_GRID_SIZE);

	// Применяем основную адаптацию.
	if (CFG.G2EnableAdaptReact) {
		AdvanceRPM += get_interpolated_value_int16_t(TCU.DrumRPMDelta, GRIDS.DeltaRPMGrid, ADAPT.Gear2AdvAdaptGraph, DELTA_RPM_GRID_SIZE);
	}

	// Применяем коррекцию по температуре.
	AdvanceRPM += get_interpolated_value_int16_t(TCU.OilTemp, GRIDS.TempGrid, TABLES.Gear2AdvTempCorrGraph, TEMP_GRID_SIZE);

	// Применяем адаптацию по температуре.
	if (CFG.G2EnableAdaptRctTemp) {
		AdvanceRPM += get_interpolated_value_int16_t(TCU.OilTemp, GRIDS.TempGrid, ADAPT.Gear2AdvTempAdaptGraph, TEMP_GRID_SIZE);
	}

	return AdvanceRPM;
}

// Давление включения третьей передачи SLU B2.
uint16_t get_slu_pressure_gear3() {
	uint16_t SLU = get_interpolated_value_uint16_t(TCU.InstTPS, GRIDS.TPSGrid, TABLES.SLUGear3Graph, TPS_GRID_SIZE);
	// Применяем коррекцию по температуре.
	SLU = CONSTRAIN(SLU + get_slu_gear2_temp_corr(SLU), 100, 980);
	return SLU;
}

// Задержка отключения SLU при включении третьей передачи.
uint16_t get_gear3_slu_delay() {
	int16_t Delay = get_interpolated_value_uint16_t(TCU.InstTPS, GRIDS.TPSGrid, TABLES.SLUGear3DelayGraph, TPS_GRID_SIZE);

	if (CFG.G3EnableAdaptTPS) {
		Delay += get_interpolated_value_int16_t(TCU.InstTPS, GRIDS.TPSGrid, ADAPT.SLUGear3TPSAdaptGraph, TPS_GRID_SIZE);
	}

	// Коррекция задержки от температуры.
	Delay += get_interpolated_value_int16_t(TCU.OilTemp, GRIDS.TempGrid, TABLES.SLUG3DelayTempCorrGraph, TEMP_GRID_SIZE);

	if (CFG.G3EnableAdaptTemp) {
		Delay += get_interpolated_value_int16_t(TCU.OilTemp, GRIDS.TempGrid, ADAPT.SLUGear3TempAdaptGraph, TEMP_GRID_SIZE);
	}

	if (Delay < 0) {return 0;}
	else {return Delay;}
}

// Смещение времени включения SLN при включении третьей передачи.
int16_t get_gear3_sln_offset() {
	int16_t Offset = get_interpolated_value_int16_t(TCU.InstTPS, GRIDS.TPSGrid, TABLES.SLNGear3OffsetGraph, TPS_GRID_SIZE);
	return Offset;
}

// Возвращает левый индекс из сетки ДПДЗ.
uint8_t get_tps_index(uint8_t TPS) {
	if (TPS >= GRIDS.TPSGrid[TPS_GRID_SIZE - 1]) {return TPS_GRID_SIZE - 2;}

	for (uint8_t i = 1; i < TPS_GRID_SIZE; i++) {
		if (TPS < GRIDS.TPSGrid[i]) {return i - 1;}
	}
	return 0;
}

// Возвращает левый индекс из сетки температуры.
uint8_t get_temp_index(int16_t Temp) {
	if (Temp >= GRIDS.TempGrid[TEMP_GRID_SIZE - 1]) {return TEMP_GRID_SIZE - 2;}

	for (uint8_t i = 1; i < TEMP_GRID_SIZE; i++) {
		if (Temp < GRIDS.TempGrid[i]) {return i - 1;}
	}
	return 0;
}

// Возвращает левый индекс из сетки дельты оборотов.
uint8_t get_delta_rpm_index(int16_t DeltaRPM) {
	if (DeltaRPM >= GRIDS.DeltaRPMGrid[DELTA_RPM_GRID_SIZE - 1]) {return DELTA_RPM_GRID_SIZE - 2;}

	for (uint8_t i = 1; i < DELTA_RPM_GRID_SIZE; i++) {
		if (DeltaRPM < GRIDS.DeltaRPMGrid[i]) {return i - 1;}
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

// Расчёт значения адаптации для одной точки.
static int16_t get_cell_adapt_step(uint8_t N, int16_t Value, int16_t LeftCell, int8_t GridStep, int16_t AdaptStep) {
	if (!N) {Value = LeftCell * 2 + GridStep - Value;}

	AdaptStep *= 4;
	AdaptStep += (GridStep - ABS((Value - LeftCell) * 2 - GridStep)) * AdaptStep / GridStep;

	int16_t Result = ((Value - LeftCell) * 32) / GridStep;
	Result = (Result * AdaptStep) / 128;
	return Result;
}

// Сохранение адаптации давления включения второй передачи.
void save_gear2_slu_adaptation(int8_t Value, uint8_t TPS) {
	uint8_t Index = 0;
	int8_t AdaptStep = 0;
	int8_t GridStep = 0;

	// Адаптация по ДПДЗ.
	if (TCU.OilTemp >= CFG.G2AdaptTPSTempMin && TCU.OilTemp <= CFG.G2AdaptTPSTempMax) {
		if (CFG.G2EnableAdaptTPS) {
			Index = get_tps_index(TPS);
			AdaptStep = 2 * CFG.AdaptationStepRatio;
			GridStep = GRIDS.TPSGrid[Index + 1] - GRIDS.TPSGrid[Index];

			ADAPT.SLUGear2TPSAdaptGraph[Index] += Value * get_cell_adapt_step(0, TPS, GRIDS.TPSGrid[Index], GridStep, AdaptStep);
			ADAPT.SLUGear2TPSAdaptGraph[Index + 1] += Value * get_cell_adapt_step(1, TPS, GRIDS.TPSGrid[Index], GridStep, AdaptStep);

			ADAPT.SLUGear2TPSAdaptGraph[Index] = CONSTRAIN(ADAPT.SLUGear2TPSAdaptGraph[Index], -32, 32);
			ADAPT.SLUGear2TPSAdaptGraph[Index + 1] = CONSTRAIN(ADAPT.SLUGear2TPSAdaptGraph[Index + 1], -32, 32);

			TCU.AdaptationTPS = Value * ADAPT_FLAG_STATE_COUNT;
		}
	}
	else {		// Адаптация по температуре масла.
		if (CFG.G2EnableAdaptTemp) {
			if (TPS > CFG.G2AdaptTempMaxTPS) {return;} // Адаптация по температуре только на малом газу.
			Index = get_temp_index(TCU.OilTemp);
			AdaptStep = 5 * CFG.AdaptationStepRatio;
			GridStep = GRIDS.TempGrid[Index + 1] - GRIDS.TempGrid[Index];

			ADAPT.SLUGear2TempAdaptGraph[Index] += Value * get_cell_adapt_step(0, TCU.OilTemp, GRIDS.TempGrid[Index], GridStep, AdaptStep);
			ADAPT.SLUGear2TempAdaptGraph[Index + 1] += Value * get_cell_adapt_step(1, TCU.OilTemp, GRIDS.TempGrid[Index], GridStep, AdaptStep);

			ADAPT.SLUGear2TempAdaptGraph[Index] = CONSTRAIN(ADAPT.SLUGear2TempAdaptGraph[Index], -120, 120);
			ADAPT.SLUGear2TempAdaptGraph[Index + 1] = CONSTRAIN(ADAPT.SLUGear2TempAdaptGraph[Index + 1], -120, 120);

			TCU.AdaptationTemp = Value * ADAPT_FLAG_STATE_COUNT;
		}
	}
}

// Сохранение адаптации опережения реактивации второй передачи.
void save_gear2_adv_adaptation(int8_t Value, int16_t InitDrumRPMDelta) {
	if (InitDrumRPMDelta < CFG.G2AdaptReactMinDRPM)	{return;}

	// Дельта оборотов может выходить за пределы сетки.
	InitDrumRPMDelta = CONSTRAIN(InitDrumRPMDelta, GRIDS.DeltaRPMGrid[0], GRIDS.DeltaRPMGrid[DELTA_RPM_GRID_SIZE - 1]);

	uint8_t Index = 0;
	int16_t AdaptStep = 25 * CFG.AdaptationStepRatio;
	int8_t GridStep = 0;

	// Адаптация по ускорению первичного вала.
	if (TCU.OilTemp >= CFG.G2AdaptReactTempMin && TCU.OilTemp <= CFG.G2AdaptReactTempMax) {
		if (CFG.G2EnableAdaptReact) {
			Index = get_delta_rpm_index(InitDrumRPMDelta);
			GridStep = GRIDS.DeltaRPMGrid[Index + 1] - GRIDS.DeltaRPMGrid[Index];

			ADAPT.Gear2AdvAdaptGraph[Index] += Value * get_cell_adapt_step(0, InitDrumRPMDelta, GRIDS.DeltaRPMGrid[Index], GridStep, AdaptStep);
			ADAPT.Gear2AdvAdaptGraph[Index + 1] += Value * get_cell_adapt_step(1, InitDrumRPMDelta, GRIDS.DeltaRPMGrid[Index], GridStep, AdaptStep);

			ADAPT.Gear2AdvAdaptGraph[Index] = CONSTRAIN(ADAPT.Gear2AdvAdaptGraph[Index], -300, 300);
			ADAPT.Gear2AdvAdaptGraph[Index + 1] = CONSTRAIN(ADAPT.Gear2AdvAdaptGraph[Index + 1], -300, 300);

			TCU.AdaptationTPS = Value * ADAPT_FLAG_STATE_COUNT;
		}
	}
	else {		// Адаптация по температуре масла.
		if (CFG.G2EnableAdaptRctTemp) {
			if (TCU.InstTPS > CFG.G2AdaptRctTempMaxTPS) {return;}
			Index = get_temp_index(TCU.OilTemp);	// 3
			GridStep = GRIDS.TempGrid[Index + 1] - GRIDS.TempGrid[Index];

			ADAPT.Gear2AdvTempAdaptGraph[Index] += Value * get_cell_adapt_step(0, TCU.OilTemp, GRIDS.TempGrid[Index], GridStep, AdaptStep);
			ADAPT.Gear2AdvTempAdaptGraph[Index + 1] += Value * get_cell_adapt_step(1, TCU.OilTemp, GRIDS.TempGrid[Index], GridStep, AdaptStep);

			ADAPT.Gear2AdvTempAdaptGraph[Index] = CONSTRAIN(ADAPT.Gear2AdvTempAdaptGraph[Index], -300, 300);
			ADAPT.Gear2AdvTempAdaptGraph[Index + 1] = CONSTRAIN(ADAPT.Gear2AdvTempAdaptGraph[Index + 1], -300, 300);

			TCU.AdaptationTemp = Value * ADAPT_FLAG_STATE_COUNT;
		}
	}
}

// Сохранение адаптации времени удержания SLU третьей передачи.
void save_gear3_slu_adaptation(int8_t Value, uint8_t TPS) {
	uint8_t Index = 0;
	int8_t AdaptStep = 12 * CFG.AdaptationStepRatio;
	int8_t GridStep = 0;

	if (Value < 0) {AdaptStep = 6 * CFG.AdaptationStepRatio;}

	// Адаптация по ДПДЗ.
	if (TCU.OilTemp >= CFG.G3AdaptTPSTempMin && TCU.OilTemp <= CFG.G3AdaptTPSTempMax) {
		if (CFG.G3EnableAdaptTPS) {
			Index = get_tps_index(TPS);
			GridStep = GRIDS.TPSGrid[Index + 1] - GRIDS.TPSGrid[Index];

			ADAPT.SLUGear3TPSAdaptGraph[Index] += Value * get_cell_adapt_step(0, TPS, GRIDS.TPSGrid[Index], GridStep, AdaptStep);
			ADAPT.SLUGear3TPSAdaptGraph[Index + 1] += Value * get_cell_adapt_step(1, TPS, GRIDS.TPSGrid[Index], GridStep, AdaptStep);

			ADAPT.SLUGear3TPSAdaptGraph[Index] = CONSTRAIN(ADAPT.SLUGear3TPSAdaptGraph[Index], -200, 200);
			ADAPT.SLUGear3TPSAdaptGraph[Index + 1] = CONSTRAIN(ADAPT.SLUGear3TPSAdaptGraph[Index + 1], -200, 200);

			TCU.AdaptationTPS = Value * ADAPT_FLAG_STATE_COUNT;
		}
	}
	else {		// Адаптация по температуре масла.
		if (CFG.G3EnableAdaptTemp) {
			if (TPS > CFG.G3AdaptTempMaxTPS) {return;} // Адаптация по температуре только на малом газу.

			Index = get_temp_index(TCU.OilTemp);
			GridStep = GRIDS.TempGrid[Index + 1] - GRIDS.TempGrid[Index];

			ADAPT.SLUGear3TempAdaptGraph[Index] += Value * get_cell_adapt_step(0, TCU.OilTemp, GRIDS.TempGrid[Index], GridStep, AdaptStep);
			ADAPT.SLUGear3TempAdaptGraph[Index + 1] += Value * get_cell_adapt_step(1, TCU.OilTemp, GRIDS.TempGrid[Index], GridStep, AdaptStep);

			ADAPT.SLUGear3TempAdaptGraph[Index] = CONSTRAIN(ADAPT.SLUGear3TempAdaptGraph[Index], -200, 200);
			ADAPT.SLUGear3TempAdaptGraph[Index + 1] = CONSTRAIN(ADAPT.SLUGear3TempAdaptGraph[Index + 1], -200, 120);

			TCU.AdaptationTemp = Value * ADAPT_FLAG_STATE_COUNT;
		}
	}
}