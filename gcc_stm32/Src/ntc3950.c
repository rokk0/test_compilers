#include "ntc3950.h"

int16_t round_float(float val)
{
  if (val < 0)
    return (int16_t)(val - 0.5);
  else
    return (int16_t)(val + 0.5);
}

float ntc3950_read_temperature(uint16_t adc_value)
{
  float average = 0;
  float temperature = -123;

  if (adc_value > TEMPERATURE_ADC_MIN && adc_value < TEMPERATURE_ADC_MAX)
  {
    average = adc_value;
    average = ((ADC_RESOLUTION - 1) / average) - 1;
    average = SERIES_RESISTOR / average;

    //Calculate temperature using the Beta Factor equation
    temperature = average / TERMISTOR_NOMINAL;            // (R/Ro)
    temperature = log(temperature);                       // ln(R/Ro)
    temperature /= BETA_VALUE;                            // 1/B * ln(R/Ro)
    temperature += 1.0 / (TEMPERATURE_NOMINAL + 273.15);  // + (1/To)
    temperature = 1.0 / temperature;                      // Invert the value
    temperature -= 273.15;                                // Convert it to Celsius
  }

  return temperature;
}
