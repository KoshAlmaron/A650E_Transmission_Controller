#include <stdint.h>				// Коротние название int.
#include <avr/io.h>				// Названия регистров и номера бит.

#include "macros.h"				// Макросы.
#include "pinout.h"				// Начначение выводов контроллера.

#include "buttons.h"			// Свой заголовок.

/*
	Состояние кнопок:
		0 - не нажата,
		1..99 - ожидание после нажатия,
		100 - событие обработано,
		101..149 - откат после срабатывания,
		201 - короткое нажатие,
		202 - длинное нажатие.
*/
static uint8_t ButtonState[2] = {};

// Прототипы функций.
static void button_read(uint8_t State, uint8_t N);

void buttons_init() {
	// Настраиваем выводы для кнопок.
	SET_PIN_MODE_INPUT(TIP_GEAR_UP_PIN);
	SET_PIN_HIGH(TIP_GEAR_UP_PIN);
	SET_PIN_MODE_INPUT(TIP_GEAR_DOWN_PIN);
	SET_PIN_HIGH(TIP_GEAR_DOWN_PIN);
}

void buttons_clear() {
	if (PIN_READ(TIP_GEAR_UP_PIN) && ButtonState[BTN_UP] > 200) {ButtonState[BTN_UP] = 100;}
	if (PIN_READ(TIP_GEAR_DOWN_PIN) && ButtonState[BTN_DOWN] > 200) {ButtonState[BTN_DOWN] = 100;}
}

// Вызов каждые 25 мс.
void buttons_update() {
	button_read(BTN_UP, PIN_READ(TIP_GEAR_UP_PIN));
	button_read(BTN_DOWN, PIN_READ(TIP_GEAR_DOWN_PIN));
}

uint8_t buttons_get_state(uint8_t N) {
	uint8_t Result = 0;
	switch (ButtonState[N]) {
		case 201:	// Короткое нажатие.
			Result = 1;
			break;
		case 202:	// Длинное нажатие.	
			Result = 2;
			break;
	}
	ButtonState[N] = 100;
	return Result;
}

static void button_read(uint8_t N, uint8_t State) {
	if (ButtonState[N] < 100) {
		if (!State) {
			ButtonState[N]++;
			if (ButtonState[N] >= 60) {ButtonState[N] = 202;}	// Длиное нажатие.
		}
		else {
			if (ButtonState[N] >= 2) {ButtonState[N] = 201;}	// Короткое нажатие.
		}
		return;
	}

	if (ButtonState[N] < 150 && State) {
		ButtonState[N]++;
		if (ButtonState[N] >= 105) {
			ButtonState[N] = 0;		// Сброс состояния кнопки.
		}
		return;		
	}
}