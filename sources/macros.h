// Макросы.

#ifndef _MACROS_H_
	#define _MACROS_H_
	
	#define MIN(a, b) ((a)<(b)?(a):(b))
	#define MAX(a, b) ((a)>(b)?(a):(b))
	#define ABS(x) ((x)>0?(x):-(x))
	#define CONSTRAIN(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


	// Макросы для работы с целыми портами.
	#define SET_PORT_MODE(port, data) (DDR ## port = data)
	#define PORT_WRITE(port, data) (PORT ## port= data)
	#define PORT_TOGGLE(port) ((PORT ## port) = ~(PORT ## port))
	#define PORT_READ(port) PIN ## port

	// Макросы для работы с конкретным выводом.
	#define SET_PIN_MODE_OUTPUT(ARGS) _SET_PIN_MODE_OUTPUT(ARGS)
	#define _SET_PIN_MODE_OUTPUT(port, pin) DDR ## port |= (1 << pin)

	#define SET_PIN_MODE_INPUT(ARGS) _SET_PIN_MODE_INPUT(ARGS)
	#define _SET_PIN_MODE_INPUT(port, pin) DDR ## port &= ~(1 << pin)

	#define SET_PIN_HIGH(ARGS) _SET_PIN_HIGH(ARGS)
	#define _SET_PIN_HIGH(port, pin) (PORT ## port |= (1 << pin))

	#define SET_PIN_LOW(ARGS) _SET_PIN_LOW(ARGS)
	#define _SET_PIN_LOW(port, pin) ((PORT ## port) &= ~(1 << (pin)))

	#define PIN_TOGGLE(ARGS) _PIN_TOGGLE(ARGS)
	#define _PIN_TOGGLE(port, pin) (PORT ## port) ^= (1 << (pin))

	#define PIN_READ(ARGS) _PIN_READ(ARGS)
	#define _PIN_READ(port, pin) (PIN ## port & (1 << pin))

#endif
/*
	https://we.easyelectronics.ru/Soft/preprocessor-c.html
	Если параметры макроса непосредственно те токены, что нам нужно склеить,
	то всё сработает как надо. Если-же это макросы, которые сначала нужно раскрыть,
	то придется вводить еще один вспомогательный макрос,
	который сначала развернёт параметры и передаст их следующему макросу.
*/