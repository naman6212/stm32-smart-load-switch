# STM32 Smart Load Switch (eFuse)

Firmware for an electronic fuse / smart load switch on an **STM32 Nucleo-C031C6** (Cortex-M0+), built with the stm32duino core and simulated in [Wokwi](https://wokwi.com).

The firmware senses load current, drives a gate output and runs a latched protection state machine. If it detects over-current, a stuck-on switch, an illegal internal state or a watchdog reset, it turns the gate off.

## Features

- **Current sensing**: 12-bit ADC on PA0, 8-sample power-of-two moving average, fixed-point conversion to mA (6600 mA full scale, i.e. 3.3 V at 0.5 V/A)
- **Protection state machine**: `IDLE → ARMED → RUN`, with a latched `FAULT` state
- **Button**: one interrupt-driven, debounced button on PA1 to arm, run, stop and reset
- **Watchdog**: independent watchdog (IWDG), about 250 ms, configured at register level; a watchdog reboot comes up in `FAULT`
- **Telemetry**: checksummed UART frames at 115200 baud
- **Display**: 16x2 I2C LCD shows current and state
- **Scheduler**: non-blocking cooperative tasks at 10 ms (sense + FSM), 200 ms (telemetry) and 250 ms (LCD)

## State machine

| State | Gate | Transitions |
|-------|------|-------------|
| `IDLE`  | off | press → `ARMED`; current > 200 mA → `FAULT` (stuck-on) |
| `ARMED` | off | press → `RUN`; 5 s timeout → `IDLE`; current > 200 mA → `FAULT` (stuck-on) |
| `RUN`   | on  | press → `IDLE`; current > 3300 mA → `FAULT` (over-current) |
| `FAULT` | off, fault LED on | press **and** current < 200 mA → `IDLE` |

The gate pin is written in exactly one place (`enterState`), so its value always follows the current state.

### Fault codes

| Code | Meaning |
|------|---------|
| 0 | None |
| 1 | Over-current |
| 2 | Stuck-on (current flowing with gate off) |
| 3 | Internal (illegal state) |
| 4 | Watchdog reset |

## Telemetry format

```
$I,<mA>,<state>,<fault>*<XX>\n
```

`XX` is the XOR of every character between `$` and `*`, written as two uppercase hex digits (NMEA style).

## Wiring

| Signal | Connection |
|--------|------------|
| Current-sense stand-in (slide pot) | VCC → 3.3V, GND → GND, SIG → PA0 (A0) |
| Button | PA1 → pushbutton → GND (internal pull-up) |
| Gate (load switch stand-in) | PA5 (onboard LED) |
| Fault LED | PA6 → 220 Ω → red LED → GND |
| LCD (I2C, 0x27) | SDA → D14, SCL → D15, VCC → 3.3V, GND → GND |

## Files

| File | Purpose |
|------|---------|
| [`sketch.ino`](sketch.ino) | Firmware source |
| [`diagram.json`](diagram.json) | Wokwi circuit (board, pot, button, LED, LCD and their wiring) |
| [`libraries.txt`](libraries.txt) | Wokwi library list (`LiquidCrystal I2C`) |

## Running it

1. Create a new STM32 project on [wokwi.com](https://wokwi.com) (Nucleo-C031C6).
2. Replace `sketch.ino` and `diagram.json` with the files in this repo, and add `libraries.txt`.
3. Start the simulation. Slide the pot to change the sensed current and press the button to arm and run.
4. Watch the serial monitor for telemetry frames and the LCD for status.
