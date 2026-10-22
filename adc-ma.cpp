// ADC sensing with Moving-Average


#include<Arduino.h>

const uint32_t SENSE_PIN = PA0;
const uint32_t ADC_BITS = 12;
const uint32_t FULL_SCALE_MA = 6600;
const uint32_t SAMPLE_PERIOD_MS = 10;
const uint32_t PRINT_PERIOD_MS = 200;

const uint32_t AVG_LEN = 8;
const uint32_t AVG_SHIFT = 3;

static_assert((AVG_LEN & (AVG_LEN - 1)) == 0, "AVG_LEN must be a power of 2");

// Filter State
static uint16_t samples[AVG_LEN];
static uint32_t sampleSum = 0;
static uint8_t sampleIdx = 0;

// ADC cod -> mA, fixed point, rounded to nearest
uint32_t codeToMilliamps(uint32_t code) {
  return (code * FULL_SCALE_MA + (1UL << (ADC_BITS - 1))) >> ADC_BITS;
}

// Take one sample, update the MA, return the filtered code
uint32_t senseUpdate() {
  uint16_t raw = analogRead(SENSE_PIN);
  sampleSum -= samples[sampleIdx]; // drop oldest
  samples[sampleIdx] = raw;
  sampleSum += raw;
  sampleIdx = (sampleIdx + 1) & (AVG_LEN - 1);
  return sampleSum >> AVG_SHIFT; 
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(ADC_BITS);
}

void loop() {
  static uint32_t lastSampleMs = 0;
  static uint32_t lastPrintMs  = 0;
  static uint32_t filtCode     = 0;
  uint32_t now = millis();

  // Sample every 10 ms
  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    filtCode = senseUpdate();
  }

  // report every 200 ms
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    Serial.print("code = "); Serial.print(filtCode);
    Serial.print(" I = "); Serial.print(codeToMilliamps(filtCode));
    Serial.println(" mA");
  }
}
