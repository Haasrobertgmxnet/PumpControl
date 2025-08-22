/*
 * Tinyadc_rawPump.cpp – ATtiny25/45/85
 * - Sensor: analog an Pin 2  -> PB3 / ADC3
 * - Pumpe:  digital an Pin 6 -> PB1 (active HIGH)
 * - Schlaf: Power-Down; Aufwachen über WDT-Interrupt
 *
 * Kompilieren mit AVR-GCC (Microchip Studio).
 * Fuses-Empfehlung (typisch): interner RC, CKDIV8 = ON (1 MHz) oder OFF (8 MHz) je nach Bedarf.
 */

#define F_CPU 1000000UL  // falls CKDIV8 aktiv ist (Standard). Sonst auf 8000000UL setzen.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "adc.h"
#include "adc_calib.h"

// --------------------------- Pinbelegung ---------------------------
constexpr uint8_t PIN_PUMP   = PB1; // phys. Pin 6
constexpr uint8_t PIN_SENSOR = PB3; // phys. Pin 2, ADC3

// --------------------------- Parameter -----------------------------

// Messintervall (Sekunden). WDT läuft mit 8s-Ticks; wird unten in Ticks umgerechnet.
constexpr uint16_t MEASUREMENT_PERIOD_S = 60;    // alle 60s messen (Praxis: 30–300s)

// Hysterese-Schwellwerte in ADC-Ticks (0..1023). Bitte kalibrieren!
// Beispiel: trocken < 420 -> Pumpe AN; feucht > 520 -> Pumpe AUS
constexpr uint16_t THRESHOLD_DRY  = 420;
constexpr uint16_t THRESHOLD_WET  = 520;

// Sicherheitsbegrenzung: maximale Pumpenlaufzeit pro Bewässerungszyklus (Sekunden)
constexpr uint16_t PUMP_MAX_ON_S  = 60;

// ADC-Mittelung für robustere Messwerte
constexpr uint8_t  ADC_SAMPLES    = 8;

// --------------------------- Globale Zustände -----------------------
volatile uint16_t wdt_ticks = 0; // zählt 8s-Ticks

// --------------------------- Utilities ------------------------------
static inline void pump_on()  { PORTB |=  (1 << PIN_PUMP); }
static inline void pump_off() { PORTB &= ~(1 << PIN_PUMP); }
static inline bool pump_is_on(){ return (PORTB & (1 << PIN_PUMP)); }

// Watchdog auf 8s-Interrupt konfigurieren
void wdt_init_8s_interrupt()
{
    cli();
    // Reset-Flag löschen
    MCUSR &= ~(1 << WDRF);
    // Timed sequence
    WDTCR = (1 << WDCE) | (1 << WDE);
    // WDIE = 1 (Interrupt), WDE = 0 (kein Reset), Prescaler = 8s (WDP3|WDP0)
    WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
    sei();
}

// In Power-Down schlafen, BOD währenddessen aus (wenn verfügbar)
void sleep_powerdown()
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
#if defined(BODS) && defined(BODSE)
    // BOD im Sleep abschalten (spart zusätzlich Strom)
    cli();
    MCUCR |= (1 << BODSE) | (1 << BODS);
    MCUCR  = (MCUCR & ~(1 << BODSE)) | (1 << BODS);
    sei();
#endif
    sleep_enable();
    sleep_cpu();
    sleep_disable();
}

// WDT-ISR: zählt die 8s-Ticks hoch
ISR(WDT_vect)
{
    wdt_ticks++;
}

// --------------------------- Hauptlogik -----------------------------

int main()
{
    // PB1 als Ausgang für Pumpe, initial AUS
    DDRB  |=  (1 << PIN_PUMP);
    pump_off();

    // Unbenötigte Peripherie sparen
    PRR |= (1 << PRUSI);   // USI aus
    PRR |= (1 << PRTIM0);  // Timer0 aus (wir nutzen WDT)
    // ADC zunächst aus (wird nur bei Messung eingeschaltet)
    adc_off();

    // WDT für periodisches Aufwachen
    wdt_init_8s_interrupt();

    sei();

    const uint16_t ticks_per_measure = (MEASUREMENT_PERIOD_S + 7) / 8; // Aufrunden
    const uint16_t pump_max_ticks    = (PUMP_MAX_ON_S + 7) / 8;        // 8s-Ticks

    // values from measurement
	uint16_t adc_1 = 550;
	uint16_t adc_2 = 250;
	uint16_t val_1 = 0;
	uint16_t val_2 = 100;
	
	//uint16_t slope = -1;	
	//uint16_t offset = 1000;	
	
	AdcCalibMaintainer<uint16_t> adcCalibMaintainer(adc_1, val_1, adc_2, val_2);
	auto adcCalib = adcCalibMaintainer.Create(1023);
	
	uint16_t up_bound = adcCalib.physToAdc(0);
	uint16_t lo_bound = adcCalib.physToAdc(100);
	
	uint16_t threshold_on = adcCalib.physToAdc(45);
	uint16_t threshold_off = adcCalib.physToAdc(55);
	
    while (true)
    {
        // --- Schlafen bis zur nächsten Messung ---
        wdt_ticks = 0;
        while (wdt_ticks < ticks_per_measure) {
            sleep_powerdown();   // wacht alle 8s kurz via ISR auf
        }

        // --- Messen ---
        adc_on();
        uint16_t adc_raw = adc_read_avg(ADC_SAMPLES);
        adc_off();

		if(adc_raw > up_bound){
			// Re-Kalibrierung
			continue;
		}
		
		if(adc_raw< lo_bound){
			// Re-Kalibrierung
			continue;
		}
		
        // --- Regelung mit Hysterese ---
        if (!pump_is_on() && (adc_raw < THRESHOLD_DRY)) {
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

                bool reached_wet = (m > THRESHOLD_WET);
                bool timeout     = (on_ticks >= pump_max_ticks);

                if (reached_wet || timeout) break;
            }

            pump_off(); // aus Sicherheitsgründen immer ausschalten
        }
        else if (pump_is_on() && (adc_raw > THRESHOLD_WET)) {
            pump_off();
        }

        // Danach geht’s automatisch in die nächste Mess-Schleife (Sleep)
    }
}
