// I2C (TWI).

#ifndef _I2C_H_
	#define _I2C_H_

	void i2c_init();
	void i2c_send_array(uint8_t* Array, uint16_t ArrSize);
	uint8_t i2c_ready();
	uint8_t i2c_get_status();


	/*** Статусные коды TWI модуля. ***/
	
	#define TWSR_MASK 			0xFC	// Маска получения кода состояния.

	// Общие статусные коды.
	#define TWI_START			0x08	// Передан сигнал START.
	#define TWI_REP_START		0x10	// Передан сигнал REPEATED START.
	#define TWI_ARB_LOST		0x38	// был потерян приоритет.

	// Статусные коды ведущего передатчика.
	#define TWI_MT_ADR_ACK		0x18	// Передан SLA+W, принят ACK.
	#define TWI_MT_ADR_NACK		0x20	// Передан SLA+W, принят NACK.
	#define TWI_MT_DATA_ACK		0x28	// Передан байт данных, принят ACK.
	#define TWI_MT_DATA_NACK	0x30	// Передан байт данных, принят NACK.

	// Статусные коды ведущего приемника.
	#define TWI_MR_ADR_ACK		0x40	// Передан SLA+R, принят ACK.
	#define TWI_MR_ADR_NACK		0x48	// Передан SLA+R, принят NACK.
	#define TWI_MR_DATA_ACK		0x50	// Принят байт данных, возвращен ACK.
	#define TWI_MR_DATA_NACK	0x58	// Принят байт данных, возвращен NACK.

	// Другие статусные коды.
	#define TWI_LOH_STATE		0x38	// Проигран арбитраж на передаче SLA+W или байт данных.
	#define TWI_NO_STATE		0xF8	// Нет доступной информации о состоянии, TWINT=0.
	#define TWI_BUS_ERROR		0x00	// Ошибка шины из-за недопустимого появления сигналов START или STOP.

	// Пользовательские коды.
	#define TWI_SUCCESS			0xFF

#endif