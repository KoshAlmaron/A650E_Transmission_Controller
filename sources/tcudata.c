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
	.RawOIL = 0
};

uint8_t SpeedTestFlag = 0;	// Флаг включения тестирования скорости.

// Прототипы локальных функций.
static uint16_t get_car_speed();

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
	return get_interpolated_value_int16_t(TempValue, OilTempGraph, TempGrid, TEMP_GRID_SIZE);
}

// Положение дросселя.
void calc_tps() {
	// ДПДЗ на ADC1.
	static uint8_t Counter = 0;

	int16_t TempValue = get_adc_value(1);
	TCU.RawTPS = TempValue;
	TCU.InstTPS = get_interpolated_value_int16_t(TempValue, TPSGraph, TPSGrid, TPS_GRID_SIZE);

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
	uint16_t SLT = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLTGraph, TPS_GRID_SIZE);
	// Применяем коррекцию по температуре.
	SLT = CONSTRAIN(SLT + get_slt_temp_corr(SLT), 80, 980);
	return SLT;
}

// Возращает коррекцию в процентах или сразу рассчитанную добавку,
// если передать функции базовое значение.
int16_t get_slt_temp_corr(int16_t Value) {
	int32_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLTTempCorrGraph, TEMP_GRID_SIZE);
	
	if (!Value) {return OilTempCorr;}	// Возвращаем коррекцию в %.
	else {	// Возвращаем скорректированное значение.
		OilTempCorr = (Value * OilTempCorr) / 1024;	// Коррекция в значениях ШИМ.
		return OilTempCorr;
	}
}

uint16_t get_sln_pressure() {
	// Вычисляем значение в зависимости от ДПДЗ.
	uint16_t SLN = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLNGraph, TPS_GRID_SIZE);
	return SLN;
}

uint16_t get_sln_pressure_gear3() {
	// Вычисляем значение в зависимости от ДПДЗ.
	uint16_t SLN = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLNGear3Graph, TPS_GRID_SIZE);
	return SLN;
}

// Давление включения и работы второй передачи SLU B3.
uint16_t get_slu_pressure_gear2() {
	uint16_t SLU = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLUGear2Graph, TPS_GRID_SIZE);
	
	if (CFG.G2EnableAdaptTPS) {
		SLU += get_interpolated_value_int16_t(TCU.InstTPS, TPSGrid, SLUGear2TPSAdaptGraph, TPS_GRID_SIZE);
	}

	// Применяем коррекцию по температуре.
	SLU = CONSTRAIN(SLU + get_slu_gear2_temp_corr(SLU), 100, 980);
	return SLU;
}

// Возращает коррекцию в процентах или сразу рассчитанную добавку,
// если передать функции базовое значение.
int16_t get_slu_gear2_temp_corr(int16_t Value) {
	int32_t OilTempCorr = get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLUGear2TempCorrGraph, TEMP_GRID_SIZE);

	if (CFG.G2EnableAdaptTemp) {
		OilTempCorr += get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLUGear2TempAdaptGraph, TEMP_GRID_SIZE);
	}

	if (!Value) {return OilTempCorr;}	// Возвращаем коррекцию в %.
	else {	// Возвращаем скорректированное значение.
		OilTempCorr = (Value * OilTempCorr) / 1024;    // Коррекция в значениях ШИМ.
		return OilTempCorr;
	}
}

// Опережение по оборотам реактивации второй передачи.
int16_t get_gear2_rpm_adv() {
	int16_t AdvanceRPM = get_interpolated_value_int16_t(TCU.DrumRPMDelta, DeltaRPMGrid, Gear2AdvGraph, DELTA_RPM_GRID_SIZE);

	// Применяем основную адаптацию.
	if (CFG.G2EnableAdaptReact) {
		AdvanceRPM += get_interpolated_value_int16_t(TCU.DrumRPMDelta, DeltaRPMGrid, Gear2AdvAdaptGraph, DELTA_RPM_GRID_SIZE);
	}

	// Применяем коррекцию по температуре.
	AdvanceRPM += get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, Gear2AdvTempCorrGraph, TEMP_GRID_SIZE);

	// Применяем адаптацию по температуре.
	if (CFG.G2EnableAdaptRctTemp) {
		AdvanceRPM += get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, Gear2AdvTempAdaptGraph, TEMP_GRID_SIZE);
	}

	return AdvanceRPM;
}

// Давление включения третьей передачи SLU B2.
uint16_t get_slu_pressure_gear3() {
	uint16_t SLU = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLUGear3Graph, TPS_GRID_SIZE);
	// Применяем коррекцию по температуре.
	SLU = CONSTRAIN(SLU + get_slu_gear2_temp_corr(SLU), 100, 980);
	return SLU;
}

// Задержка отключения SLU при включении третьей передачи.
uint16_t get_gear3_slu_delay() {
	int16_t Delay = get_interpolated_value_uint16_t(TCU.InstTPS, TPSGrid, SLUGear3DelayGraph, TPS_GRID_SIZE);

	if (CFG.G3EnableAdaptTPS) {
		Delay += get_interpolated_value_int16_t(TCU.InstTPS, TPSGrid, SLUGear3TPSAdaptGraph, TPS_GRID_SIZE);
	}

	// Коррекция задержки от температуры.
	Delay += get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLUG3DelayTempCorrGraph, TEMP_GRID_SIZE);

	if (CFG.G3EnableAdaptTemp) {
		Delay += get_interpolated_value_int16_t(TCU.OilTemp, TempGrid, SLUGear3TempAdaptGraph, TEMP_GRID_SIZE);
	}

	if (Delay < 0) {return 0;}
	else {return Delay;}
}

// Смещение времени включения SLN при включении третьей передачи.
int16_t get_gear3_sln_offset() {
	int16_t Offset = get_interpolated_value_int16_t(TCU.InstTPS, TPSGrid, SLNGear3OffsetGraph, TPS_GRID_SIZE);
	return Offset;
}

// Возвращает левый индекс из сетки ДПДЗ.
uint8_t get_tps_index(uint8_t TPS) {
	if (TPS == TPSGrid[TPS_GRID_SIZE - 1]) {return TPS_GRID_SIZE - 2;}

	for (uint8_t i = 1; i < TPS_GRID_SIZE; i++) {
		if (TPS < TPSGrid[i]) {return i - 1;}
	}
	return 0;
}

// Возвращает левый индекс из сетки температуры.
uint8_t get_temp_index(int16_t Temp) {
	if (Temp == TempGrid[TEMP_GRID_SIZE - 1]) {return TEMP_GRID_SIZE - 2;}

	for (uint8_t i = 1; i < TEMP_GRID_SIZE; i++) {
		if (Temp < TempGrid[i]) {return i - 1;}
	}
	return 0;
}

// Возвращает левый индекс из сетки дельты оборотов.
uint8_t get_delta_rpm_index(int16_t DeltaRPM) {
	if (DeltaRPM == DeltaRPMGrid[DELTA_RPM_GRID_SIZE - 1]) {return DELTA_RPM_GRID_SIZE - 2;}

	for (uint8_t i = 1; i < DELTA_RPM_GRID_SIZE; i++) {
		if (DeltaRPM < DeltaRPMGrid[i]) {return i - 1;}
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

void save_gear2_slu_adaptation(int8_t Value) {
	uint8_t Index = 0;
	int8_t Step = 0;
	int16_t Add = 0;

	// Адаптация по ДПДЗ.
	if (TCU.OilTemp >= CFG.G2AdaptTPSTempMin && TCU.OilTemp <= CFG.G2AdaptTPSTempMax) {
		if (CFG.G2EnableAdaptTPS) {
			Index = get_tps_index(TCU.InstTPS);		// 2
			Step = 2 * CFG.AdaptationStepRatio;		// 2

			Add = 32 - ((TCU.InstTPS - TPSGrid[Index]) * 32) / 5;	// 13
			Add += 16 / Step;	// 21
			Add *= Step;		// 42
			Add /= 32;			// 1

			SLUGear2TPSAdaptGraph[Index] += Add * Value;
			SLUGear2TPSAdaptGraph[Index + 1] += (Step - Add) * Value;

			SLUGear2TPSAdaptGraph[Index] = CONSTRAIN(SLUGear2TPSAdaptGraph[Index], -32, 32);
			SLUGear2TPSAdaptGraph[Index + 1] = CONSTRAIN(SLUGear2TPSAdaptGraph[Index + 1], -32, 32);
		}
	}
	else {		// Адаптация по температуре масла.
		if (CFG.G2EnableAdaptTemp) {
			if (TCU.InstTPS > CFG.G2AdaptTempMaxTPS) {return;} // Адаптация по температуре только на малом газу.
			Index = get_temp_index(TCU.OilTemp);
			Step = 5 * CFG.AdaptationStepRatio;

			Add = 32 - ((TCU.OilTemp - TempGrid[Index]) * 32) / 5;
			Add += 16 / Step;
			Add *= Step;
			Add /= 32;

			SLUGear2TempAdaptGraph[Index] += Add * Value;
			SLUGear2TempAdaptGraph[Index + 1] += (Step - Add) * Value;

			SLUGear2TempAdaptGraph[Index] = CONSTRAIN(SLUGear2TempAdaptGraph[Index], -120, 120);
			SLUGear2TempAdaptGraph[Index + 1] = CONSTRAIN(SLUGear2TempAdaptGraph[Index + 1], -120, 120);
		}
	}
}

void save_gear2_adv_adaptation(int8_t Value, int16_t InitDrumRPMDelta) {
	if (InitDrumRPMDelta < CFG.G2AdaptReactMinDRPM)	{return;}

	uint8_t Index = 0;
	int8_t Step = 20 * CFG.AdaptationStepRatio;
	int16_t Add = 0;

	// Адаптация по ДПДЗ.
	if (TCU.OilTemp >= CFG.G2AdaptReactTempMin && TCU.OilTemp <= CFG.G2AdaptReactTempMax) {
		if (CFG.G2EnableAdaptReact) {
			Index = get_delta_rpm_index(InitDrumRPMDelta);
			Add = 32 - ((InitDrumRPMDelta - DeltaRPMGrid[Index]) * 32) / 5;

			Add += 16 / Step;
			Add *= Step;
			Add /= 32;

			Gear2AdvAdaptGraph[Index] += Add * Value;
			Gear2AdvAdaptGraph[Index + 1] += (Step - Add) * Value;

			Gear2AdvAdaptGraph[Index] = CONSTRAIN(Gear2AdvAdaptGraph[Index], -300, 300);
			Gear2AdvAdaptGraph[Index + 1] = CONSTRAIN(Gear2AdvAdaptGraph[Index + 1], -300, 300);
		}
	}
	else {		// Адаптация по температуре масла.
		if (CFG.G2EnableAdaptRctTemp) {
			if (TCU.InstTPS > CFG.G2AdaptRctTempMaxTPS) {return;}
			Index = get_temp_index(TCU.OilTemp);	// 3

			Add = 32 - ((TCU.OilTemp - TempGrid[Index]) * 32) / 5;
			Add += 16 / Step;
			Add *= Step;
			Add /= 32;

			Gear2AdvTempAdaptGraph[Index] = CONSTRAIN(Gear2AdvTempAdaptGraph[Index], -300, 300);
			Gear2AdvTempAdaptGraph[Index + 1] = CONSTRAIN(Gear2AdvTempAdaptGraph[Index + 1], -300, 300);
		}
	}
}

void save_gear3_slu_adaptation(int8_t Value) {
	uint8_t Index = 0;
	int8_t Step = 12 * CFG.AdaptationStepRatio;
	int16_t Add = 0;

	if (Value < 0) {Step = 6;}

	// Адаптация по ДПДЗ.
	if (TCU.OilTemp >= CFG.G3AdaptTPSTempMin && TCU.OilTemp <= CFG.G3AdaptTPSTempMax) {
		if (CFG.G3EnableAdaptTPS) {

			Index = get_tps_index(TCU.InstTPS);

			Add = 32 - ((TCU.InstTPS - TPSGrid[Index]) * 32) / 5;
			Add += 16 / Step;
			Add *= Step;
			Add /= 32;

			SLUGear3TPSAdaptGraph[Index] += Add * Value;
			SLUGear3TPSAdaptGraph[Index + 1] += (Step - Add) * Value;

			SLUGear3TPSAdaptGraph[Index] = CONSTRAIN(SLUGear3TPSAdaptGraph[Index], -200, 200);
			SLUGear3TPSAdaptGraph[Index + 1] = CONSTRAIN(SLUGear3TPSAdaptGraph[Index + 1], -200, 200);
		}
	}
	else {		// Адаптация по температуре масла.
		if (CFG.G3EnableAdaptTemp) {
			if (TCU.InstTPS > CFG.G3AdaptTempMaxTPS) {return;} // Адаптация по температуре только на малом газу.

			Index = get_temp_index(TCU.OilTemp);

			Add = 32 - ((TCU.OilTemp - TempGrid[Index]) * 32) / 5;
			Add += 16 / Step;
			Add *= Step;
			Add /= 32;

			SLUGear3TempAdaptGraph[Index] += Add * Value;
			SLUGear3TempAdaptGraph[Index + 1] += (Step - Add) * Value;

			SLUGear3TempAdaptGraph[Index] = CONSTRAIN(SLUGear3TempAdaptGraph[Index], -200, 200);
			SLUGear3TempAdaptGraph[Index + 1] = CONSTRAIN(SLUGear3TempAdaptGraph[Index + 1], -200, 120);
		}
	}
}