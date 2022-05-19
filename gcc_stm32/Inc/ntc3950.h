#ifndef __NTC3950_H
#define __NTC3950_H

#include <math.h>
#include <stdint.h>

// Nominal resistance value for the thermistor
#define TERMISTOR_NOMINAL 10000
// Nominl temperature value for the thermistor
#define TEMPERATURE_NOMINAL 25
// Beta value for our thermistor
#define BETA_VALUE 3950
// Value of the series resistor
#define SERIES_RESISTOR 10000
// ADC resolution
#define ADC_RESOLUTION 4096
// LIMITS
#define TEMPERATURE_ADC_MAX       4000    // Consider NTC is disconnected, if above this level (4000 ADC == -41 C)
#define TEMPERATURE_ADC_MIN       10      // Consider NTC is short, if below this level (10 ADC == 273 C)

float ntc3950_read_temperature(uint16_t adc_value);
int16_t round_float(float val);

#endif
