volatile unsigned long pulses = 0;       // contador de pulsos por ISR
unsigned long timeBef = 0;
float flow = 0.0;        // L/min
float litersTotal = 0.0; // litros acumulados

// Fator de calibração calculado
const float Kf = 5.12;  // L/min por Hz
const float Q0 = 0.0;   // offset, geralmente 0

void IRAM_ATTR pulseISR() {
  pulses++;
}

void setup() {
  Serial.begin(115200);
  pinMode(19, INPUT);  // pino do sinal do sensor
  attachInterrupt(digitalPinToInterrupt(19), pulseISR, RISING);

  Serial.println("Medidor de Vazão e Volume - ESP32");
  timeBef = millis();
}

void loop() {
  unsigned long now = millis();

  // calcula a cada 1 segundo
  if (now - timeBef >= 1000) {
    // lê e zera os pulsos de forma segura
    noInterrupts();
    unsigned long pulsePeriod = pulses;
    pulses = 0;
    interrupts();

    float interval_s = (now - time_bef) / 1000.0;

    // frequência em Hz
    float freq = pulsePeriod / interval_s;

    // vazão L/min
    flow = (Kf * freq) + Q0;

    // volume do período (L)
    float litersPeriod = flow * (interval_s / 60.0); // fluxo em L/min -> volume em L
    liters += litersPeriod;

    // exibe no Serial
    Serial.print("Freq (Hz): "); Serial.print(freq, 3);
    Serial.print(" | Fluxo (L/min): "); Serial.print(flow, 3);
    Serial.print(" | Volume período (L): "); Serial.print(litersPeriod, 4);
    Serial.print(" | Volume total (L): "); Serial.println(litersTotal, 4);

    // atualiza tempo
    timeBef = now;
  }
}
