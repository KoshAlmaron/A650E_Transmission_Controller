// PacAT
// Назчначение выводов контроллера.

#ifndef _PINOUT_H_
	#define _PINOUT_H_
	
	#define SOLENOID_S1_PIN H, 3
	#define SOLENOID_S2_PIN H, 4
	#define SOLENOID_S3_PIN H, 5
	#define SOLENOID_S4_PIN H, 6

	#define SELECTOR_P_PIN C, 1
	#define SELECTOR_R_PIN C, 3
	#define SELECTOR_N_PIN C, 4
	#define SELECTOR_D_PIN G, 5
	#define SELECTOR_3_PIN K, 6
	#define SELECTOR_2_PIN C, 2
	#define SELECTOR_4_PIN K, 4
	#define SELECTOR_L_PIN D, 7

	#define TIP_GEAR_UP_PIN G, 2
	#define TIP_GEAR_DOWN_PIN C, 7

	#define BREAK_PEDAL_PIN G, 1
	#define REAR_LAMP_PIN B, 4

	#define ENGINE_WORK_PIN A, 7
	#define REQUEST_POWER_DOWN_PIN J, 1

	// ============== DEBUG ==============
	#define DEBUG_LCD_ON_PIN A, 0
	#define DEBUG_MODE_ON_PIN A, 1
	#define DEBUG_S1_PIN A, 2
	#define DEBUG_S2_PIN A, 3
	#define DEBUG_S3_PIN A, 4
	#define DEBUG_S4_PIN A, 5

	#define DEBUG_SCREEN_BUTTON_F_PIN A, 6
	#define DEBUG_SCREEN_BUTTON_R_PIN A, 7
	
	// ============== DEBUG ==============
	//Нештатный код/patch
	#define SPEED2_OUT_PIN E, 5
#endif


/*
при обновлении:	
configuration.h - Полностью
pinout.h - Полностью
tculogic.c - логика задней if (TCU.CarSpeed < 10 && (TCU.Break || TCU.ATMode == 1 || TCU.ATMode == 3)) {
tcudata_tables.h - таблица давлений и ацп
gears_tables.h - характеристика переключений
		SET_PIN_MODE_INPUT(SPEED_2_PIN);
		SET_PIN_LOW(SPEED_2_PIN);
		в main.c
и остальные грязные хаки (2 UART, 2 скорости, тахометр)

	Arduino MEGA 2560

	0 PE0	(RX0)			| USB UART            | UART0 USB RX
	1 PE1	(TX0)			| USB UART            | UART0 USB TX	 
   ~2 PE4	(OC3B) (INT4)	| INPUT_TACHO_LOGIC   | Вход тахометра
   ~3 PE5	(OC3C) (INT5)	| OUT_SPEED_2_LOGIC   | [ОБРЕЗАН] Выход скорости 2 (на круиз)
   ~4 PG5	(OC0B)			| INPUT_D_LOGIC       | Положение селектора АКПП "D".
   ~5 PE3	(OC3A)			| OUT_SPEED_1_LOGIC   | Выход скорости 1 (на спидометр)
   ~6 PH3	(OC4A)			| OUT_S1_LOGIC        | Соленоид S1.
   ~7 PH4	(OC4B)			| OUT_S2_LOGIC        | Соленоид S2.
   ~8 PH5	(OC4C)			| OUT_S3_LOGIC        | Соленоид S3.
   ~9 PH6	(OC2B)			| OUT_S4_LOGIC        | Соленоид S4.
   ~10 PB4	(OC2A)			| OUT_R_LOGIC         | [НЕ ПОДКЛЮЧЕНО] Лампа заднего хода.
   ~11 PB5	(OC1A)			| OUT_SLT_LOGIC       | Соленоид SLT.
   ~12 PB6	(OC1B)			| OUT_SLN_LOGIC       | Соленоид SLN.
   ~13 PB7	(OC1C)			| OUT_SLU_LOGIC       | Соленоид SLU.
	14 PJ1	(TX3)			| OUT_NC+_LOGIC       | !!! Запрос снижения мощности.
	15 PJ0	(RX3)			|                     | 
	16 PH1	(TX2)			| CAN TX              | UART2 CAN TX
	17 PH0	(RX2)			| CAN RX              | UART2 CAN RX
	18 PD3	(TX1) (INT3)	| BLUETOOTH TX        | !!! UART1 BLUETOOTH TX (Продублировать UART0)
	19 PD2	(RX1) (INT2)	| BLUETOOTH RX        | !!! UART1 BLUETOOTH RX (Продублировать UART0)
	20 PD1	(SDA) (INT1)	|                     | I2C LCD отладки.
	21 PD0	(SCL) (INT0)	|                     | I2C LCD отладки.
	22 PA0					|                     | Включение экрана отладки.
	23 PA1					|                     | Включение ручного управления соленоидами.
	24 PA2					|                     | Тумблер ручного управления S1.
	25 PA3					|                     | Тумблер ручного управления S2.
	26 PA4					|                     | Тумблер ручного управления S3.
	27 PA5					|                     | Тумблер ручного управления S4.
	28 PA6					|	
	29 PA7					|					  | [NEW] Определение работы двигателя, HI-работа
	30 PC7					| INPUT_TIP-_LOGIC    |
	31 PC6					|                     | [ОТСУТСТВУЕТ] Положение селектора АКПП "D4".
	39 PG2					| INPUT_TIP+_LOGIC    |
	33 PC4					| INPUT_N_LOGIC       | Положение селектора АКПП "N".
	34 PC3					| INPUT_R_LOGIC       | Положение селектора АКПП "R".
	35 PC2					| INPUT_2_LOGIC       | Положение селектора АКПП "2".
	36 PC1					| INPUT_P_LOGIC       | Положение селектора АКПП "P".
	37 PC0					| INPUT_REZERV3_LOGIC |	[ОТСУТСТВУЕТ] резервный оптронный канал CH3
	38 PD7					| INPUT_L_LOGIC	      | Положение селектора АКПП "L".	
	40 PG1					| INPUT_BRAKE_LOGIC   | Педаль тормоза
	32 PC5					| INPUT_M_LOGIC       | 
	41 PG0					|                     | 
	42 PL7					|                     | 
	43 PL6					|                     | 
   ~44 PL5	(OC5C)			|                     | 
   ~45 PL4	(OC5B)			| OUT_LED-_1_LOGIC    | [НЕ ПОДКЛЮЧЕНО] Выход на индикатор перегрева (+)
   ~46 PL3	(OC5A)			| OUT_LED-_2_LOGIC    | Выход на индикатор (+)
	47 PL2					| OUT_LED-_3_LOGIC    | Выход на индикатор (+)
	48 PL1	(ICP5)			| OUTSHAFT_TRIG       | Датчик скорости выходного вала.
	49 PL0	(ICP4)			| INSHAFT_TRIG        | Датчик скорости корзины овердрайва.
	50 PB3	(MISO)			|                     | SPI.
	51 PB2	(MOSI)			|                     | SPI.
	52 PB1	(SCK)			|                     | SPI.
	53 PB0	(SS)			|                     | SPI.

	A0 PF0	(ADC0)			| IN_T_LOGIC          | Датчик температуры масла.
	A1 PF1	(ADC1)			| IN_DPDZ_LOGIC       | ДПДЗ.
	A2 PF2	(ADC2)			|                     | 
	A3 PF3	(ADC3)			|                     | 
	A4 PF4	(ADC4)			|                     | 
	A5 PF5	(ADC5)			|                     | 
	A6 PF6	(ADC6)			|                     | 
	A7 PF7	(ADC7)			|                     | 
	A8 PK0	(ADC8)			|                     | CYCLE_TEST_PIN
	A9 PK1	(ADC9)			|                     | 
	A10 PK2	(ADC10)			|                     | 
	A11 PK3	(ADC11)			|                     | Ручное управление SLT.
	A12 PK4	(ADC12)			|                     | Ручное управление SLN / Кнопка смены экрана отладки назад.
	A13 PK5	(ADC13)			|                     | Ручное управление SLU / Кнопка смены экрана отладки	вперед.
	A14 PK6	(ADC14)			|                     | PCINT22 переместить сюда тахометр	
	A15 PK7	(ADC15)			|                     | 	
*/