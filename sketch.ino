// =====================================================================
// STM32 Smart Load Switch (eFuse) Firmware
// Board: STM32 Nucleo-C031C6 (Cortex-M0+), simulated in Wokwi (stm32duino core)
//
// Features:
//   - 12-bit ADC current sensing, 8-sample moving-average filter, fixed-point mA
//   - Latched protection state machine: IDLE / ARMED / RUN / FAULT
//       fault 1 = over-current, 2 = stuck-on (current with gate off),
//       fault 3 = internal (illegal state), 4 = watchdog reset
//   - Interrupt-driven, debounced arm / run / stop / reset button
//   - Independent watchdog (IWDG, ~250 ms), configured at register level
//   - Checksummed UART telemetry:  $I,<mA>,<state>,<fault>*<XOR checksum>\n
//   - 16x2 I2C LCD status display
//   - Non-blocking cooperative scheduler (10 ms / 200 ms / 250 ms tasks)
//
// Wiring:
//   Pot (current-sense stand-in): VCC -> 3.3V, GND -> GND, SIG -> PA0 (A0)
//   Button:     PA1 -> pushbutton -> GND
//   Gate:       PA5 = onboard LED (load switch stand-in)
//   Fault LED:  PA6 -> 220 ohm -> red LED -> GND
//   LCD (I2C):  SDA -> board SDA, SCL -> board SCL, VCC -> 3.3V, GND -> GND
//
// Library: LiquidCrystal I2C
// =====================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "stm32yyxx_ll_rcc.h"

// ── Pins (bit numbers on port A) ────────────────────────────────────
const uint32_t GATE_PIN      = 5;    // PA5
const uint32_t FAULT_LED_PIN = 6;    // PA6
const uint32_t BTN_PIN       = 1;    // PA1, active-low

// ── Sensing configuration ───────────────────────────────────────────
const uint32_t SENSE_PIN     = PA0;  // pot = simulated amplifier output
const uint32_t ADC_BITS      = 12;
const uint32_t FULL_SCALE_MA = 6600; // 3.3 V / 0.5 V/A = 6.6 A
const uint32_t AVG_LEN       = 8;    // must be a power of 2
const uint32_t AVG_SHIFT     = 3;    // log2(AVG_LEN)
static_assert((AVG_LEN & (AVG_LEN - 1)) == 0, "AVG_LEN must be a power of 2");

// ── Protection configuration ────────────────────────────────────────
const uint32_t TRIP_LIMIT_MA  = 3300; // RUN: above this -> over-current fault
const uint32_t OFF_CURRENT_MA = 200;  // gate OFF: above this -> stuck-on fault
                                      // FAULT: reset allowed only below this
const uint32_t ARM_TIMEOUT_MS = 5000; // ARMED falls back to IDLE after this
const uint32_t QUIET_MS       = 20;   // button debounce window

// ── Watchdog configuration ──────────────────────────────────────────
const uint32_t WDG_RELOAD = 249;      // LSI 32 kHz / 32 = 1 kHz -> 250 ticks ~ 250 ms

// ── Scheduler periods ───────────────────────────────────────────────
const uint32_t SAMPLE_PERIOD_MS = 10;
const uint32_t PRINT_PERIOD_MS  = 200;
const uint32_t LCD_PERIOD_MS    = 250;

// ── Telemetry and LCD ───────────────────────────────────────────────
const uint32_t TELEM_BUF_LEN = 48;
const uint8_t  LCD_ADDR      = 0x27;

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

static const char HEX_DIGITS[] = "0123456789ABCDEF";

// ── State machine types ─────────────────────────────────────────────
enum State : uint8_t { ST_IDLE, ST_ARMED, ST_RUN, ST_FAULT };

enum FaultCode : uint8_t {
  FAULT_NONE        = 0,
  FAULT_OVERCURRENT = 1,
  FAULT_STUCK_ON    = 2,
  FAULT_INTERNAL    = 3,
  FAULT_WATCHDOG    = 4
};

// ── State machine state ─────────────────────────────────────────────
static State     state        = ST_IDLE;
static FaultCode faultCode    = FAULT_NONE;
static uint32_t  stateEntryMs = 0;

volatile bool pressEvent = false;     // set by ISR, consumed by loop()

// ── Filter state ────────────────────────────────────────────────────
static uint16_t samples[AVG_LEN];     // circular buffer
static uint32_t sampleSum = 0;        // invariant: sampleSum == sum of samples[]
static uint8_t  sampleIdx = 0;

// =====================================================================
// GPIO helpers (register level)
// =====================================================================

static inline void pinWrite(uint32_t pin, bool high) {
  GPIOA->BSRR = high ? (1UL << pin) : (1UL << (pin + 16));
}

static inline bool pinRead(uint32_t pin) {
  return (GPIOA->IDR >> pin) & 1UL;
}

static void pinMakeOutput(uint32_t pin) {
  pinWrite(pin, false);                        // set value LOW first...
  GPIOA->MODER &= ~(0x3UL << (pin * 2));
  GPIOA->MODER |=  (0x1UL << (pin * 2));       // ...then enable output: no glitch
}

static void pinMakeInputPullup(uint32_t pin) {
  GPIOA->MODER &= ~(0x3UL << (pin * 2));       // input
  GPIOA->PUPDR &= ~(0x3UL << (pin * 2));
  GPIOA->PUPDR |=  (0x1UL << (pin * 2));       // pull-up
}

// =====================================================================
// Watchdog (IWDG, register level)
// =====================================================================

void wdgStart() {
  IWDG->KR  = 0xCCCC;              // start (also starts the LSI clock)
  IWDG->KR  = 0x5555;              // unlock PR and RLR
  IWDG->PR  = 3;                   // prescaler: LSI / 32
  IWDG->RLR = WDG_RELOAD;
  while (IWDG->SR != 0) { }        // wait for settings to reach the IWDG clock domain
  IWDG->KR  = 0xAAAA;              // first kick
}

static inline void wdgKick() {
  IWDG->KR = 0xAAAA;
}

bool wasWatchdogReset() {
  bool flag = LL_RCC_IsActiveFlag_IWDGRST();
  LL_RCC_ClearResetFlags();        // clear so the next boot reads fresh
  return flag;
}

// =====================================================================
// Text formatting (replaces snprintf to save flash)
// =====================================================================

// Write v as decimal text; return pointer just past the last digit
static char *appendUint(char *dst, uint32_t v) {
  char tmp[10];                    // uint32_t max is 10 digits
  uint8_t n = 0;
  do {
    tmp[n++] = '0' + (v % 10);     // lowest digit first
    v /= 10;
  } while (v != 0);
  while (n > 0) {
    *dst++ = tmp[--n];             // copy back in the correct order
  }
  return dst;
}

// Copy a string (without its '\0'); return pointer past the end
static char *appendStr(char *dst, const char *s) {
  while (*s) {
    *dst++ = *s++;
  }
  return dst;
}

// Write v as exactly two uppercase hex digits
static char *appendHex2(char *dst, uint8_t v) {
  *dst++ = HEX_DIGITS[v >> 4];
  *dst++ = HEX_DIGITS[v & 0x0F];
  return dst;
}

// Fill with spaces to 16 characters so old LCD text is overwritten
static void padTo16(char *line, char *end) {
  while (end < line + 16) {
    *end++ = ' ';
  }
  *end = '\0';
}

// =====================================================================
// Current sensing
// =====================================================================

// ADC code -> milliamps, fixed-point, rounded to nearest
static uint32_t codeToMilliamps(uint32_t code) {
  return (code * FULL_SCALE_MA + (1UL << (ADC_BITS - 1))) >> ADC_BITS;
}

// Configure the ADC and pre-fill the filter with a real reading
void senseInit() {
  analogReadResolution(ADC_BITS);          // the core defaults to 10-bit
  uint16_t first = analogRead(SENSE_PIN);
  for (uint32_t i = 0; i < AVG_LEN; i++) {
    samples[i] = first;
  }
  sampleSum = (uint32_t)first * AVG_LEN;
}

// Take one sample, update the moving average, return filtered current in mA
uint32_t senseUpdate() {
  uint16_t raw = analogRead(SENSE_PIN);
  sampleSum -= samples[sampleIdx];         // drop oldest
  samples[sampleIdx] = raw;
  sampleSum += raw;                        // add newest
  sampleIdx = (sampleIdx + 1) & (AVG_LEN - 1);
  return codeToMilliamps(sampleSum >> AVG_SHIFT);
}

// =====================================================================
// Protection state machine
// =====================================================================

const char *stateName(State s) {
  switch (s) {
    case ST_IDLE:  return "IDLE";
    case ST_ARMED: return "ARMED";
    case ST_RUN:   return "RUN";
    case ST_FAULT: return "FAULT";
    default:       return "???";
  }
}

// The ONLY place the gate pin is written. Moore outputs set on entry.
static void enterState(State next, uint32_t now) {
  state        = next;
  stateEntryMs = now;
  pinWrite(GATE_PIN,      next == ST_RUN);
  pinWrite(FAULT_LED_PIN, next == ST_FAULT);
}

static void enterFault(FaultCode code, uint32_t now) {
  faultCode = code;
  enterState(ST_FAULT, now);
}

// Called every 10 ms with the latest filtered current and button event
void fsmUpdate(uint32_t currentMa, bool press, uint32_t now) {
  switch (state) {

    case ST_IDLE:
      if (currentMa > OFF_CURRENT_MA)      enterFault(FAULT_STUCK_ON, now);
      else if (press)                      enterState(ST_ARMED, now);
      break;

    case ST_ARMED:
      if (currentMa > OFF_CURRENT_MA)      enterFault(FAULT_STUCK_ON, now);
      else if (press)                      enterState(ST_RUN, now);
      else if (now - stateEntryMs >= ARM_TIMEOUT_MS)
                                           enterState(ST_IDLE, now);
      break;

    case ST_RUN:
      if (currentMa > TRIP_LIMIT_MA)       enterFault(FAULT_OVERCURRENT, now);
      else if (press)                      enterState(ST_IDLE, now);
      break;

    case ST_FAULT:                         // latched: reset needs a press AND near-zero current
      if (press && currentMa < OFF_CURRENT_MA) {
        faultCode = FAULT_NONE;
        enterState(ST_IDLE, now);
      }
      break;

    default:                               // corrupted state: fail safe
      enterFault(FAULT_INTERNAL, now);
      break;
  }
}

// =====================================================================
// Telemetry
// =====================================================================

// XOR of every character (NMEA-style checksum)
static uint8_t nmeaChecksum(const char *s) {
  uint8_t cs = 0;
  while (*s) {
    cs ^= (uint8_t)*s;
    s++;
  }
  return cs;
}

// Max length: $ I, 10 digits , 5-char state , 3 digits * 2 hex \n \0 = 28 < TELEM_BUF_LEN
void telemetrySend(uint32_t currentMa, const char *stateStr, uint8_t fault) {
  char packet[TELEM_BUF_LEN];
  char *p = packet;

  *p++ = '$';
  char *payload = p;                       // checksum covers the text after '$'
  *p++ = 'I';
  *p++ = ',';
  p = appendUint(p, currentMa);
  *p++ = ',';
  p = appendStr(p, stateStr);
  *p++ = ',';
  p = appendUint(p, fault);
  *p = '\0';                               // terminate payload for the checksum

  uint8_t cs = nmeaChecksum(payload);
  *p++ = '*';
  p = appendHex2(p, cs);
  *p++ = '\n';
  *p = '\0';

  Serial.print(packet);
}

// =====================================================================
// LCD display
// =====================================================================

void displayInit() {
  Wire.begin();
  lcd.init();
  lcd.backlight();
}

void displayUpdate(uint32_t currentMa, const char *stateStr) {
  char line[17];                           // 16 characters + '\0'
  char *p;

  p = appendStr(line, "I=");
  p = appendUint(p, currentMa);
  p = appendStr(p, " mA");
  padTo16(line, p);
  lcd.setCursor(0, 0);
  lcd.print(line);

  p = appendStr(line, "State: ");
  p = appendStr(p, stateStr);
  padTo16(line, p);
  lcd.setCursor(0, 1);
  lcd.print(line);
}

// =====================================================================
// Button ISR: leading-edge debounce, triggered on both edges
// =====================================================================

void onButtonEdge() {
  static uint32_t lastEdgeMs = 0;          // only used here, so no volatile needed
  uint32_t now = millis();
  bool pinLow = !pinRead(BTN_PIN);

  if ((now - lastEdgeMs) >= QUIET_MS && pinLow) {
    pressEvent = true;                     // real press: tell main
  }
  lastEdgeMs = now;                        // every edge restarts the quiet window
}

// =====================================================================
// Setup and main loop
// =====================================================================

void setup() {
  // 1. Safe outputs FIRST: gate and fault LED forced off before anything else
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  pinMakeOutput(GATE_PIN);
  pinMakeOutput(FAULT_LED_PIN);

  // 2. Peripherals
  Serial.begin(115200);
  senseInit();
  attachInterrupt(digitalPinToInterrupt(PA1), onButtonEdge, CHANGE);
  pinMakeInputPullup(BTN_PIN);             // after attachInterrupt so it isn't overridden
  displayInit();

  // 3. Starting state: IDLE normally, FAULT if the watchdog rebooted us
  if (wasWatchdogReset()) {
    enterFault(FAULT_WATCHDOG, millis());
  } else {
    enterState(ST_IDLE, millis());
  }

  // 4. Watchdog LAST, once all slow setup is done
  wdgStart();
}

void loop() {
  static uint32_t lastSampleMs = 0;
  static uint32_t lastPrintMs  = 0;
  static uint32_t lastLcdMs    = 0;
  static uint32_t currentMa    = 0;
  uint32_t now = millis();

  // Task 1 (10 ms): sense, then run the state machine on fresh data
  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    currentMa = senseUpdate();

    noInterrupts();                        // atomic read-and-clear of the ISR flag
    bool press = pressEvent;
    pressEvent = false;
    interrupts();

    fsmUpdate(currentMa, press, now);
  }

  // Task 2 (200 ms): telemetry
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    telemetrySend(currentMa, stateName(state), faultCode);
  }

  // Task 3 (250 ms): display
  if (now - lastLcdMs >= LCD_PERIOD_MS) {
    lastLcdMs = now;
    displayUpdate(currentMa, stateName(state));
  }

  wdgKick();                               // proof of life: the whole loop completed
}