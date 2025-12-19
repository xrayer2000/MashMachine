#include <Adafruit_ADS1X15.h> //ADC
long lastReadTime = 0;
Adafruit_ADS1115 adc;
int16_t adc1_raw = 13100;
int16_t adc2_raw = 0;
int16_t filteredAdc1 = adc1_raw;
int16_t filteredAdc2 = adc2_raw;
//NTC Thermistor
const double Vref = 3.27; //power supply voltage (3.3 V rail) -STM32 ADC pin is NOT 5 V tolerant
double Vout; //Voltage divider output
double R_NTC = 104300.0; //NTC thermistor, Resistance at 25 °C
const double R_ref = 101900.0; //100k resistor measured resistance in Ohms (other element in the voltage divider)
const double BETA = 3950.0; //B-coefficient of the thermistor
const double T_0 = 298.15; //25°C in Kelvin
double Temp_C; //Temperature measured by the thermistor (Celsius)

void setup() {
  //=================================================SETUP=======================================================
  Serial.begin(115200);

  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);

  //setupADC
  
  delay(10);
  adc.begin();
  delay(100);
  adc.setGain(GAIN_ONE); // 1x gain ±4.096V  1 bit = 1mV  0.0625mV
  adc.setDataRate(RATE_ADS1115_128SPS);

  adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false);
  while(!adc.conversionComplete());
  adc1_raw = adc.getLastConversionResults();
  filteredAdc1 = adc1_raw;

  adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, /*continuous=*/false);
  while(!adc.conversionComplete());
  adc2_raw = adc.getLastConversionResults();
  filteredAdc2 = adc2_raw;
}

void loop()
{
  unsigned long currentTime = millis();
    if (currentTime - lastReadTime >= 400)
    {
      lastReadTime = currentTime;

      adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false);
      while(!adc.conversionComplete());
      adc1_raw = adc.getLastConversionResults();

      filteredAdc1 = Filter(adc1_raw, filteredAdc1, 0.9); 
      Temp_C = convertRawToCelsius(filteredAdc1);
      Temp_C = roundTo(Temp_C, 1); // Round to 1 decimal place

      adc.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, /*continuous=*/false);
      while(!adc.conversionComplete());
      adc2_raw = adc.getLastConversionResults();

      filteredAdc2 = Filter(adc2_raw, filteredAdc2, 0.0); 

      Serial.print("waterRaw: ");
      Serial.print(filteredAdc2);

      Serial.print(", Temp_C: ");
      Serial.print(Temp_C, 1);
      Serial.print(", Filtered ADC1: ");
      Serial.println(filteredAdc1); 

  }
}

double Filter(double New, double Current, double alpha) //Moisture sensor
{
  return (1.0 - alpha) * New + alpha * Current;
}

double convertRawToCelsius(double raw) {
  double vOut = (raw / 32767.0) * 3.3;    //16384.0
  double rNTC = (R_ref * vOut) / (3.3 - vOut); 
  // Beta-formeln
  double invT = (1.0 / T_0) + (1.0 / BETA) * log(rNTC / R_NTC);
  double T = 1.0 / invT;    // Kelvin
  T -= 273.15;             // Celsius
  T = map(T, 15.0, 100.0, 0.0, 100.0);
  T = constrain(T, 0.0, 100.0);
  // Serial.print(", Temp_C: ");
  // Serial.println(T, 1);
  return T;       // Celsius
}

float roundTo(float value, int decimals) {
  float multiplier = powf(10.0f, decimals);
  return roundf(value * multiplier) / multiplier;
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}