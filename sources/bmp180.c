#include <stdint.h>			// Коротние название int.

#include "macros.h"			// Макросы.
#include "i2c.h"			// I2C.
#include "tcudata.h"		// Расчет и хранение всех необходимых параметров.
#include "configuration.h"	// Настройки.
#include "bmp180.h"			// Свой заголовок.

static void bmp_data_calculate();

#define BMP_ADDRESS 0x77
#define BMP_CHIPID_VALUE 0x55
#define BMP_FIRST_REGISTER 0xAA
#define BMP_CHIP_ID_REGISTER 0xD0
#define BMP_CONTROL_REGISTER 0xF4
#define BMP_DATA_REGISTER 0xF6

#define BMP_GET_TEMP_COMMAND 0x2E
#define BMP_GET_BARO_COMMAND 0x34

#define WAIT_CYCLES 25	// Количество циклов ожидания после измерений.

#define TEMPERATURE_MIN -400	// x0.1 градус.
#define TEMPERATURE_MAX 600

#define PRESSURE_MIN 800		// x0.1 кПа.
#define PRESSURE_MAX 1200

static uint8_t Buffer[8] = {0};

// Инициализация структуры с переменными.
BMP_t BMP = {
	.AC1 = 0,
	.AC2 = 0,
	.AC3 = 0,
	.AC4 = 0,
	.AC5 = 0,
	.AC6 = 0,
	.B1 = 0,
	.B2 = 0,
	.MB = 0,
	.MC = 0,
	.MD = 0,
	.ChipID = 0x55,
	.UT = 0,
	.UP = 0,
	.T = 0,
	.P = 0,
	.OSS = 1,
	.Status = 0,
	.Error = 255
};

static void bmp_data_calculate() {
	// Переносим коэффициенты в 32-х битные переменные.
	int32_t AC1 = BMP.AC1;
	int32_t AC2 = BMP.AC2;
	int32_t AC3 = BMP.AC3;
	uint32_t AC4 = BMP.AC4;
	uint32_t AC5 = BMP.AC5;
	uint32_t AC6 = BMP.AC6;
	int32_t B1 = BMP.B1;
	int32_t B2 = BMP.B2;
	int32_t MC = BMP.MC;
	int32_t MD = BMP.MD;

	// Переменные для промежуточных вычислений.
	int32_t X1 = 0;
	int32_t X2 = 0;
	int32_t X3 = 0;
	int32_t B3 = 0;
	uint32_t B4 = 0;
	int32_t B5 = 0;
	int32_t B6 = 0;
	int32_t B7 = 0;
	int32_t P = 0;

	// Температура.
	X1 = ((BMP.UT - AC6) * AC5) >> 15;
	X2 = (MC << 11) / (X1 + MD);
	B5 = X1 + X2;
	int32_t T = (B5 + 8) >> 4;
	BMP.T = T;

	// Давление.
	B6 = B5 - 4000;
	X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
	X2 = (AC2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = (((AC1 * 4 + X3) << BMP.OSS) + 2) / 4;
	X1 = (AC3 * B6) >> 13;
	X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
	X3 = (((X1 + X2) + 2) >> 2) + 32768;
	B4 = (AC4 * X3) >> 15;
	B7 = ((uint32_t) BMP.UP - B3) * (50000 >> BMP.OSS);
	P = (B7 / B4) * 2;
	X1 = (P >> 8) * (P >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * P) >> 16;
	P = P + ((X1 + X2 + 3791) >> 4);

	P /= 100;	// Переводим в 0.1 кПа.
	BMP.P = P;

	TCU.Barometer = BMP.P;
}

void bmp_proccess() {
	static uint8_t Counter = 0;
	static uint8_t WaitTimer = 0;

	if (!CFG.BaroCorrEnable) {
		Counter = 0;
		WaitTimer = 0;
		return;
	}

	WaitTimer++;
	// Сброс при зависании интерфейса или модуля.
	if (WaitTimer >= 100) {
		BMP.Status = 0;
		WaitTimer = 0;
		Counter = 0;
		i2c_stop();
	}

	if (BMP.T >= TEMPERATURE_MIN && BMP.T <= TEMPERATURE_MAX) {
		if (BMP.P >= PRESSURE_MIN && BMP.P <= PRESSURE_MAX) {
			BMP.Error = 0;
		}
		else {BMP.Error = 7;}
	}
	else {BMP.Error = 6;}

	if (!i2c_ready()) {return;}

	switch (BMP.Status) {
		case 0:			// Запрос калибровочных данных.
			// Пауза при страрте.
			Counter++;
			if (Counter < 10) {return;}
			Counter = 0;

			Buffer[0] = BMP_ADDRESS;
			Buffer[1] = BMP_FIRST_REGISTER;
			i2c_receive_data(Buffer, 2, 22);	// Команда 2 байта, ответ 22 байта.
			BMP.Status = 1;
			break;
		case 1:			// Ожидание калибровочных данных.
			if (i2c_get_rx_size() == 22) {	// Размер калибровочнных данных 22 + 1 байт.
				BMP.Status = 2;
				// Начальный адрес структуры BMP.
				uint8_t* BMPAddr = (uint8_t*) &BMP;
				// Заталкиваем в массив калибровки.
				for (uint8_t i = 0; i < 11; i++) {
					*(BMPAddr + i * 2) = i2c_get_rx_byte(i * 2 + 1);
					*(BMPAddr + i * 2 + 1) = i2c_get_rx_byte(i * 2);
				}
			}
			else {
				Counter = 0;
				BMP.Status = 0;
				BMP.Error = 1;
			}
			break;
		case 2:			// Запрос ChipID.
			Buffer[0] = BMP_ADDRESS;
			Buffer[1] = BMP_CHIP_ID_REGISTER;
			i2c_receive_data(Buffer, 2, 1);	// Команда 2 байта, ответ 1 байт.
			BMP.Status = 3;
			break;
		case 3:			// Ожидание ChipID.
			if (i2c_get_rx_size() == 1) {
			 	BMP.ChipID = i2c_get_rx_byte(0);
				if (BMP.ChipID == BMP_CHIPID_VALUE) {	// Проверка ChipID.
					BMP.Status = 4;
				}
				else {
					Counter = 0;
					BMP.Status = 0;
					BMP.Error = 3;	// ChipID не совпадает.
				}
			}
			else {
				Counter = 0;
				BMP.Status = 0;
				BMP.Error = 2;	// ChipID не получен.
			}
			break;
		case 4:			// Команда на замер температуры.
			Buffer[0] = BMP_ADDRESS;
			Buffer[1] = BMP_CONTROL_REGISTER;
			Buffer[2] = BMP_GET_TEMP_COMMAND;
			i2c_send_data(Buffer, 3);	// Команда 3 байта.
			BMP.Status = 5;
			break;
		case 5:			// Запрос регистра управления измерениями CSO.
			Buffer[1] = BMP_CONTROL_REGISTER;
			i2c_receive_data(Buffer, 2, 1);
			BMP.Status = 6;
			break;
		case 6:			// Проверка флага состояния CSO, пятый бит.
			if (i2c_get_rx_size() == 1) {
				uint8_t CSO = i2c_get_rx_byte(0);
				if (!BITREAD(CSO, 5)) {	// Ожидаем спад флага состояния CSO в 0.
					BMP.Status = 7;	// Измерение завершено.
				}
				else {
					BMP.Status = 5;	// Повторный запрос регистра CSO.
				}
			}
			break;
		case 7:			// Запрос сырого значения температуры.
			Buffer[1] = BMP_DATA_REGISTER;
			i2c_receive_data(Buffer, 2, 2);
			BMP.Status = 8;
			break;
		case 8:			// Чтение сырого значения температуры.
			if (i2c_get_rx_size() == 2) {
				uint8_t MSB = i2c_get_rx_byte(0);
				uint8_t LSB = i2c_get_rx_byte(1);
				BMP.UT = (MSB << 8) + LSB;
				BMP.Status = 9;
			}
			else {
				BMP.Status = 7;
				BMP.Error = 4;
			}
		case 9:			// Команда на замер давления.
			Buffer[0] = BMP_ADDRESS;
			Buffer[1] = BMP_CONTROL_REGISTER;
			Buffer[2] = BMP_GET_BARO_COMMAND + (BMP.OSS << 6);
			i2c_send_data(Buffer, 3);
			BMP.Status = 10;
			break;
		case 10:		// Запрос регистра управления измерениями CSO.
			Buffer[1] = BMP_CONTROL_REGISTER;
			i2c_receive_data(Buffer, 2, 1);
			BMP.Status = 11;
			break;
		case 11:		// Проверка флага состояния CSO, пятый бит.
			if (i2c_get_rx_size() == 1) {
				uint8_t CSO = i2c_get_rx_byte(0);
				if (!BITREAD(CSO, 5)) {	// Ожидаем спад флага состояния CSO в 0.
					BMP.Status = 12;	// Измерение завершено.
				}
				else {
					BMP.Status = 10;	// Повторный запрос регистра CSO.
				}
			}
			break;
		case 12:		// Запрос сырого значения давления.
			Buffer[1] = BMP_DATA_REGISTER;
			i2c_receive_data(Buffer, 2, 3);
			BMP.Status = 13;
			break;
		case 13:		// Чтение сырого значения давления.
			if (i2c_get_rx_size() == 3) {
				uint32_t MSB = i2c_get_rx_byte(0);
				uint32_t LSB = i2c_get_rx_byte(1);
				uint32_t xLSB = i2c_get_rx_byte(2);
				BMP.UP = ((MSB << 16) + (LSB << 8) + xLSB) >> (8 - BMP.OSS);
				BMP.Status = 14;
			}
			else {
				BMP.Status = 12;
				BMP.Error = 5;
			}
			break;
		case 14:		// Вычисление значений.
			bmp_data_calculate();
			BMP.Status = 15;
			break;
		case 15:		// Ожидание.
			WaitTimer = 0;	// Сброс счётчика зависания.
			Counter++;
			if (Counter >= 25) {
				Counter = 0;
				BMP.Status = 4;	// Новый цикл замера.
			}
	}
}