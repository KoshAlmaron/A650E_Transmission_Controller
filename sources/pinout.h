// Начначение выводов контроллера.

#ifndef _PINOUT_H_
	#define _PINOUT_H_
	
	#define SOLENOID_S1_PIN H, 3
	#define SOLENOID_S2_PIN H, 4
	#define SOLENOID_S3_PIN H, 5
	#define SOLENOID_S4_PIN H, 6

	#define SELECTOR_P_PIN C, 0
	#define SELECTOR_R_PIN C, 1
	#define SELECTOR_N_PIN C, 2
	#define SELECTOR_D_PIN C, 3
	#define SELECTOR_3_PIN C, 4
	#define SELECTOR_2_PIN C, 5
	#define SELECTOR_4_PIN C, 6
	#define SELECTOR_L_PIN C, 7

	#define TIP_GEAR_UP_PIN L, 7
	#define TIP_GEAR_DOWN_PIN L, 6

	#define BREAK_PEDAL_PIN G, 5
	#define REAR_LAMP_PIN B, 4

	#define ENGINE_WORK_PIN J, 0
	#define REQUEST_POWER_DOWN_PIN J, 1

	// ============== DEBUG ==============
	#define DEBUG_LCD_ON_PIN A, 0
	#define DEBUG_MODE_ON_PIN A, 1
	#define DEBUG_S1_PIN A, 2
	#define DEBUG_S2_PIN A, 3
	#define DEBUG_S3_PIN A, 4
	#define DEBUG_S4_PIN A, 5

	#define DEBUG_SCREEN_BUTTON_F_PIN K, 4
	#define DEBUG_SCREEN_BUTTON_R_PIN K, 5

	// ============== DEBUG ==============
#endif


/*
	Arduino MEGA 2560

	0 PE0	(RX0)			|	
	1 PE1	(TX0)			|	UART0 TX	 
   ~2 PE4	(OC3B) (INT4)	|	Вход сигнала тахометра двигателя.
   ~3 PE5	(OC3C) (INT5)	|
   ~4 PG5	(OC0B)			|+	Педаль тормоза.
   ~5 PE3	(OC3A)			|	Выход на спидометр
   ~6 PH3	(OC4A)			|+	Соленоид S1.
   ~7 PH4	(OC4B)			|+	Соленоид S2.
   ~8 PH5	(OC4C)			|+	Соленоид S3.
   ~9 PH6	(OC2B)			|+	Соленоид S4.
   ~10 PB4	(OC2A)			|+	Лампа заднего хода.
   ~11 PB5	(OC1A)			|+	Соленоид SLT.
   ~12 PB6	(OC1B)			|+	Соленоид SLN.
   ~13 PB7	(OC1C)			|+	Соленоид SLU.
	14 PJ1	(TX3)			|+	Запрос снижения мощности.
	15 PJ0	(RX3)			|+	Определение работы двигателя по блокировке стартера.
	16 PH1	(TX2)			|	
	17 PH0	(RX2)			|	
	18 PD3	(TX1) (INT3)	|	
	19 PD2	(RX1) (INT2)	|	
	20 PD1	(SDA) (INT1)	|+	I2C LCD отладки.
	21 PD0	(SCL) (INT0)	|+	I2C LCD отладки.
	22 PA0					|+	Включение экрана отладки.
	23 PA1					|+	Включение ручного управления соленоидами.
	24 PA2					|+	Тумблер ручного управления S1.
	25 PA3					|+	Тумблер ручного управления S2.
	26 PA4					|+	Тумблер ручного управления S3.
	27 PA5					|+	Тумблер ручного управления S4.
	28 PA6					|	
	29 PA7					|
	30 PC7					|+	Положение селектора АКПП "L".
	31 PC6					|+	Положение селектора АКПП "D4".
	32 PC5					|+	Положение селектора АКПП "2".
	33 PC4					|+	Положение селектора АКПП "3".
	34 PC3					|+	Положение селектора АКПП "D".
	35 PC2					|+	Положение селектора АКПП "N".
	36 PC1					|+	Положение селектора АКПП "R".
	37 PC0					|+	Положение селектора АКПП "P".
	38 PD7					|
	39 PG2					|
	40 PG1					|
	41 PG0					|
	42 PL7					|	Типтроник передача вверх.
	43 PL6					|	Типтроник передача вниз.
   ~44 PL5	(OC5C)			|
   ~45 PL4	(OC5B)			|
   ~46 PL3	(OC5A)			|
	47 PL2					|	
	48 PL1	(ICP5)			|+	Датчик скорости выходного вала.
	49 PL0	(ICP4)			|+	Датчик скорости корзины овердрайва.
	50 PB3	(MISO)			|	SPI.
	51 PB2	(MOSI)			|	SPI.
	52 PB1	(SCK)			|	SPI.
	53 PB0	(SS)			|	SPI.

	A0 PF0	(ADC0)			|+	Датчик температуры масла.
	A1 PF1	(ADC1)			|+	ДПДЗ.
	A2 PF2	(ADC2)			|
	A3 PF3	(ADC3)			|	
	A4 PF4	(ADC4)			|	
	A5 PF5	(ADC5)			|	
	A6 PF6	(ADC6)			|
	A7 PF7	(ADC7)			|
	A8 PK0	(ADC8)			|
	A9 PK1	(ADC9)			|	
	A10 PK2	(ADC10)			|	
	A11 PK3	(ADC11)			|+	Ручное управление SLT.
	A12 PK4	(ADC12)			|+	Ручное управление SLN / Кнопка смены экрана отладки назад.
	A13 PK5	(ADC13)			|+	Ручное управление SLU / Кнопка смены экрана отладки	вперед.
	A14 PK6	(ADC14)			|	
	A15 PK7	(ADC15)			|	
*/