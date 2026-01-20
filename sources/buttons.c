#include <stdint.h>				// Коротние название int.
#include <avr/io.h>				// Названия регистров и номера бит.

#include "macros.h"				// Макросы.
#include "pinout.h"				// Начначение выводов контроллера.

#include "buttons.h"			// Свой заголовок.

/*
	Состояние кнопок:
		0 - не нажата,
		1..99 - ожидание после нажатия,
		100..149 - откат после срабатывания,
		201 - короткое нажатие,
		202 - длинное нажатие,
		203 - событие обработано.
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

// Вызов каждые 25 мс.
void buttons_update() {
	button_read(TIP_GEAR_UP, PIN_READ(TIP_GEAR_UP_PIN));
	button_read(TIP_GEAR_DOWN, PIN_READ(TIP_GEAR_DOWN_PIN));
}

void buttons_clear() {
	if (PIN_READ(TIP_GEAR_UP_PIN) && ButtonState[TIP_GEAR_UP] > 200) {ButtonState[TIP_GEAR_UP] = 100;}
	if (PIN_READ(TIP_GEAR_DOWN_PIN) && ButtonState[TIP_GEAR_DOWN] > 200) {ButtonState[TIP_GEAR_DOWN] = 100;}
}

uint8_t is_button_press_short(uint8_t N) {
	if (ButtonState[N] == 201) {
		ButtonState[N] = 203;
		return 1;
	}
	else {return 0;}
}

uint8_t is_button_press_long(uint8_t N) {
	if (ButtonState[N] == 202) {
		ButtonState[N] = 203;
		return 1;
	}
	else {return 0;}
}

uint8_t is_button_hold_down(uint8_t N) {
	if (N == TIP_GEAR_UP && !PIN_READ(TIP_GEAR_UP_PIN)) {return 1;}
	else if (N == TIP_GEAR_DOWN && !PIN_READ(TIP_GEAR_DOWN_PIN)) {return 1;}
	else {return 0;}
}

static void button_read(uint8_t N, uint8_t State) {
	if (ButtonState[N] < 100) {
		if (!State) {
			ButtonState[N]++;
			if (ButtonState[N] >= 60) {ButtonState[N] = 202;}	// Длиное нажатие.
		}
		else {
			if (ButtonState[N] >= 2) {ButtonState[N] = 201;}	// Короткое нажатие.
			else {ButtonState[N] = 0;}
		}
		return;
	}

	if (ButtonState[N] < 150 && State) {
		ButtonState[N]++;
		if (ButtonState[N] >= 110) {
			ButtonState[N] = 0;		// Сброс состояния кнопки.
		}
		return;		
	}
}