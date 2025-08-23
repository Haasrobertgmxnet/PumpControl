/*
 * main.cpp – ATtiny25/45/85
 * - Sensor: analog pin 2  -> PB3 / ADC3
 * - Pump:  digital pin 6 -> PB1 (active HIGH)
 * - Schlaf: Power-Down; Aufwachen über WDT-Interrupt
 *
 * Compile with AVR-GCC (Microchip Studio).
 * Recommendation for typical fuses: internal RC, CKDIV8 = ON (1 MHz)
 */

#define F_CPU 1000000UL  // i.e. CKDIV8 is active

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "adc.h"
#include "adc_calib.h"
#include "adc_calib_maintainer.h"

// --------------------------- Use of pins ---------------------------
constexpr uint8_t PIN_PUMP   = PB1; // phys. Pin 6
constexpr uint8_t PIN_SENSOR = PB3; // phys. Pin 2, ADC3

// --------------------------- Parameter -----------------------------

// Measurement interval (seconds). WDT is running with 8s ticks
constexpr uint16_t MEASUREMENT_PERIOD_S = 60;    // do a measurement each 60s

// Thresholds for hysteresis in percent
constexpr uint16_t THRESHOLD_DRY  = 45; // 45 percent or lower to turn on the pump
constexpr uint16_t THRESHOLD_WET  = 55; // 45 percent or lower to turn off the pump

// For safety: maximal time to run the pump (seconds)
constexpr uint16_t PUMP_MAX_ON_S  = 60;

// Take the average of ADC_SAMPLES measurements
constexpr uint8_t  ADC_SAMPLES = 8;

// --------------------------- Global states -----------------------
volatile uint16_t wdt_ticks = 0; // counts 8s ticks

// --------------------------- Utilities ------------------------------
static inline void pump_on()  { PORTB |=  (1 << PIN_PUMP); }
static inline void pump_off() { PORTB &= ~(1 << PIN_PUMP); }
static inline bool pump_is_on(){ return (PORTB & (1 << PIN_PUMP)); }

// Configure watchdog to 8s interrupts
void wdt_init_8s_interrupt()
{
    cli();
    // delete reset flag
    MCUSR &= ~(1 << WDRF);
    // Timed sequence
    WDTCR = (1 << WDCE) | (1 << WDE);
    // WDIE = 1 (Interrupt), WDE = 0 (no Reset), prescaler = 8s (WDP3|WDP0)
    WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
    sei();
}

// sleep in  power down, where BOD is disabled during sleep period
void sleep_powerdown()
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
#if defined(BODS) && defined(BODSE)
    // turn off BOD in sleep mode
    cli();
    MCUCR |= (1 << BODSE) | (1 << BODS);
    MCUCR  = (MCUCR & ~(1 << BODSE)) | (1 << BODS);
    sei();
#endif
    sleep_enable();
    sleep_cpu();
    sleep_disable();
}

constexpr uint8_t PIN_LED = PB0;

static inline void led_on()    { PORTB |=  (1 << PIN_LED); }
static inline void led_off()   { PORTB &= ~(1 << PIN_LED); }
static inline void led_toggle(){ PINB  =   (1 << PIN_LED); } // schneller Toggle-Trick

// WDT-ISR: count the 8s ticks
ISR(WDT_vect)
{
	wdt_ticks++;
	led_toggle();  // Sichtbarer Takt: einmal pro Aufwachen
}

// --------------------------- Hauptlogik -----------------------------

int main()
{
    // PB1 als Ausgang für Pumpe, initial AUS
    DDRB  |=  (1 << PIN_PUMP);
    pump_off();

	DDRB |= (1 << PIN_LED);  // LED-Pin als Ausgang
	led_off();
	
    // Unbenötigte Peripherie sparen
    PRR |= (1 << PRUSI);   // USI aus
    PRR |= (1 << PRTIM0);  // Timer0 aus (wir nutzen WDT)
    // ADC zunächst aus (wird nur bei Messung eingeschaltet)
    adc_off();

    // WDT für periodisches Aufwachen
    wdt_init_8s_interrupt();

    sei();

    const uint16_t ticks_per_measure = (MEASUREMENT_PERIOD_S + 7) / 8; // round up
    const uint16_t pump_max_ticks    = (PUMP_MAX_ON_S + 7) / 8;        // 8s ticks

    // values from measurement
	// Here put in the values obtained from sensor calibration
	uint16_t adc_1 = 550;
	uint16_t adc_2 = 250;
	uint16_t val_1 = 0;
	uint16_t val_2 = 100;
	
	AdcCalibMaintainer<uint16_t> adcCalibMaintainer(adc_1, val_1, adc_2, val_2);
	adcCalibMaintainer.SaveToEEPROM(0);
	adcCalibMaintainer.LoadFromEEPROM(0);
	auto adcCalib = adcCalibMaintainer.Create(1023);
	
	uint16_t up_bound = adcCalib.physToAdc(0);
	uint16_t lo_bound = adcCalib.physToAdc(100);
	
	uint16_t threshold_on = adcCalib.physToAdc(THRESHOLD_DRY);
	uint16_t threshold_off = adcCalib.physToAdc(THRESHOLD_WET);
	
    while (true)
    {
        // --- Sleep until next measurement ---
        wdt_ticks = 0;
        while (wdt_ticks < ticks_per_measure) {
			led_off(); 
            sleep_powerdown();   // wacht alle 8s kurz via ISR auf
        }

        // --- Measurement ---
		led_on();    
        adc_on();
        uint16_t adc_raw = adc_read_avg(ADC_SAMPLES);
        adc_off();

		adcCalibMaintainer.LoadFromEEPROM(0);
		adcCalibMaintainer.ApplyCalibration(adcCalib);
		
		auto blink_calib = [](){
			led_on();  _delay_ms(40);
			led_off(); _delay_ms(40);
			led_on();  _delay_ms(40);
			led_off();
		};
		
		if(adc_raw > up_bound){
			adcCalibMaintainer.ReCalibrateLower(adc_raw, val_1);
			adcCalibMaintainer.SaveToEEPROM(0);
			blink_calib();
			continue;
		}
		
		if(adc_raw< lo_bound){
			adcCalibMaintainer.ReCalibrateLower(adc_raw, val_1);
			adcCalibMaintainer.SaveToEEPROM(0);
			blink_calib();
			continue;
		}
		
        // --- Regelung mit Hysterese ---
        if (!pump_is_on() && (adc_raw > threshold_on)) {
            // Pumpe einschalten
            pump_on();

            // Laufzeit begrenzen & live auf Feuchteanstieg warten
            uint16_t on_ticks = 0;
            while (true) {
                // kurze Pause, dann erneut messen (ADC kurz aktivieren)
                // Wir schlafen jeweils 8s für sehr sparsamen Betrieb.
                wdt_ticks = 0;
                sleep_powerdown();
                on_ticks++;

                adc_on();
                uint16_t m = adc_read_avg(ADC_SAMPLES);
                adc_off();

                bool reached_wet = (m < threshold_off);
                bool timeout     = (on_ticks >= pump_max_ticks);

                if (reached_wet || timeout) break;
            }

            pump_off(); // aus Sicherheitsgründen immer ausschalten
        }
        else if (pump_is_on() && (adc_raw < threshold_off)) {
            pump_off();
        }
		
		led_off();  // fertig; zurück in Sleep

        // Danach geht’s automatisch in die nächste Mess-Schleife (Sleep)
    }
}
