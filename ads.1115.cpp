#include <ADS1X15.h>

/* cria instância do conversor analógico digital ADC */
ADS1115 ads(0x49);  // endereço I2C (ajuste se necessário)

void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("ADS1X15_LIB_VERSION:");
  Serial.println(ADS1X15_LIB_VERSION);

  if (!ads.begin()) {
    Serial.println("Erro de pareamento");
  } // inicializa o ADS1115
}

void loop () {
  ads.setGain(0);  // ±6.144V

  int16_t val_0 = ads.readADC(0);
  int16_t val_1 = ads.readADC(1);
  int16_t val_2 = ads.readADC(2);
  int16_t val_3 = ads.readADC(3);

  float f = ads.toVoltage(0);  // fator de conversão para volts

  Serial.print("Analog0: "); Serial.print(val_0);
  Serial.print("\t"); Serial.println(val_0 * f, 3);

  Serial.print("Analog1: "); Serial.print(val_1);
  Serial.print("\t"); Serial.println(val_1 * f, 3);

  Serial.print("Analog2: "); Serial.print(val_2);
  Serial.print("\t"); Serial.println(val_2 * f, 3);

  Serial.print("Analog3: "); Serial.print(val_3);
  Serial.print("\t"); Serial.println(val_3 * f, 3);

  delay(1000);
}
