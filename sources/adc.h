// АЦП.

#ifndef _ADC_H_
	#define _ADC_H_

	void adc_init();
	void adc_read();
	uint16_t get_adc_value(uint8_t Channel);
	void add_channels_on(uint8_t Value);

#endif