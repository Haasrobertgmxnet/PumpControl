/*
 * adc.h
 *
 * Created: 20.08.2025 22:44:52
 *  Author: haasr
 */ 

#pragma once
//#ifndef ADC_H_
//#define ADC_H_

#include <avr/io.h>
#include <util/delay.h>

// ADC initialisieren (Vref = VCC, Kanal = ADC3/PB3)
void adc_init()
{
	// (0 << REFS1) | (0 << REFS0) : Vcc as reference voltage for the ADC
	// (3 << MUX0) : ADC channel 3 at PB3
	// (0 << ADLAR) : right aligned output from ADC. This is the default
	ADMUX = (0 << REFS1) | (0 << REFS0) | (3 << MUX0);// | (0 << ADLAR);
	
	
	// ADCSRA: ADC enable, Prescaler = 8 -> bei 1 MHz ~125 kHz
	// (1 << ADEN) : ADC enable
	// (1 << ADPS1) | (1 << ADPS0) Prescaler = 8, i.e. F_CPU / 0b11 = 1 Mhz / 8 = ~125 kHz
	ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // ADPS1:0 = 0b11 -> /8
	
	// Short setting time
	_delay_ms(2);
}

// Einzelmessung starten & Ergebnis (10 Bit) lesen
uint16_t adc_read_once()
{
	ADCSRA |= (1 << ADSC);              // Start conversion
	while (ADCSRA & (1 << ADSC)) { }    // wait ...
	
	// 10-bit Result
	uint16_t v;
	v = ADCL;        // zuerst Low-Byte lesen
	v |= ((uint16_t)ADCH << 8);  // dann High-Byte lesen und hochschieben
	return v;
}

// Gemittelte ADC-Messung
uint16_t adc_read_avg(uint8_t n = 10)
{
	// erste Messung verworfen (Multiplexer/Settling)
	(void)adc_read_once();
	uint32_t sum = 0;
	for (uint8_t i = 0; i < n; i++) {
		sum += adc_read_once();
	}
	return static_cast<uint16_t>(sum / n);
}

// ADC sauber deaktivieren (spart ~200 µA)
void adc_off()
{
	ADCSRA &= ~(1 << ADEN);
	PRR |= (1 << PRADC); // Power-Reduction für ADC einschalten
}

// ADC wieder aktivieren
void adc_on()
{
	PRR &= ~(1 << PRADC);
	adc_init();
}
//#endif /* ADC_H_ */