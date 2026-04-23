#ifndef TEMPERATURA_h
#define TEMPERATURA_h

#include <Arduino.h>

class TEMPERATURA 
{
public:

   TEMPERATURA(int pin);
           return pin; 


   void beginMedia();{
        const int NUM_MEDS;
        float historicoR[NUM_MEDS];
        int indice; 
        int contador;
   }
   float setTemperatura(float resistencia, float beta){
        float temperatura_kelvin = 1.0 / ((1.0 / T_25) + (1.0 / beta) * log(resistencia / R_25));
 
    return temperatura_kelvin - 273.15;

   }
   void mediaMovel(float beta){
        if (contador == 0) return NAN;
        float soma = 0.0;
        for (int i = 0; i < contador; i++) {
          soma += calcTemperatura(historicoR[i], beta);
        }
        return soma / contador;
   }
   
   

private:

    float VCC = 5.0;
    float R_FIXO = 10000.0;
    int ADC_MAX = 1023;
    float BETA = 3950.0;
    float R_25 = 10000.0;
    float T_25 = 25.0 + 273.15

};


#endif
// valores fixos do sensor 
//VCC, R_FIXO, ADC_MAX,Beta,R_25,T_25
//variaveis
