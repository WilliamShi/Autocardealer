# AutoCarDealer v2 — Automatic Card Dealer

An Arduino-based automatic card dealer with two DC motors. Motor A rotates the carousel to the target player position, Motor B feeds cards. Photoelectric sensors count cards and detect deals. A 16x2 LCD shows status, and two NEC infrared remotes provide control.

---

## 1. Features

- Supports 2–8 players, 1–3 decks, with or without jokers
- Supports "remaining cards" deduction (deal a fixed number per player, then stop)
- Dual infrared remotes (NEC protocol); score adjustment, stop, and resume work while running
- LCD real-time display: players, decks, jokers, score X/Y, direction, dealt/total, state character
- Red/green LED indication: stopped = red, running = green, paused = yellow (red + green)
- Motor A auto-homes on power-up; manual calibration supported
- Motor A/B timeout and Motor A stall protection; automatic pause on fault
- Non-blocking LED blink, does not block the main loop
- Optional: fast LCD refresh, missed-card compensation, per-deck pause check (see "Compile Switches")

---

## 2. Hardware Requirements

| Category | Model / Spec | Qty | Notes |
|---|---|---|---|
| MCU | Arduino Uno R3 or Nano (ATmega328P) | 1 | 5 V logic required |
| Motor driver | TB6612FNG module | 1 | Dual DC motor driver |
| Display | LCD1602 parallel (16x2) | 1 | Parallel interface required; I2C backpack not supported |
| IR receiver | VS1838B / HX1838 | 1 | 38 kHz, connected to A1 |
| IR remote | NEC protocol remote | 1–2 | Second one optional |
| Photo sensor A | Slot optocoupler / IR break-beam | 1 | Carousel 16-edge counting; LOW when blocked |
| Photo sensor B | Slot optocoupler / IR break-beam | 1 | Card-feed detection; LOW when blocked |
| Motor A | DC gear motor | 1 | Drives carousel |
| Motor B | DC gear motor | 1 | Drives feed wheel |
| LED | 5 mm red, green | 1 each | 220 Ω series resistor |
| Potentiometer | 10 kΩ | 1 | LCD contrast |
| Power | 12 V/2 A adapter + 5 V/2 A buck | 1 set | Or separate 5 V/3 A supply |
| Decoupling caps | 100 µF electrolytic + 0.1 µF ceramic | Several | See BOM |

---

## 3. Pin Connections

### 3.1 Arduino Pin Assignment

| Arduino Pin | Connected To | Direction | Notes |
|---|---|---|---|
| D2 | LCD D7 | OUT | |
| D3 | LCD D6 | OUT | |
| D4 | LCD D5 | OUT | |
| D5 | LCD D4 | OUT | |
| D6 | TB6612 AIN1 | OUT | Motor A direction |
| D7 | TB6612 AIN2 | OUT | Motor A direction |
| D8 | TB6612 STBY | OUT | Driver enable, active high |
| D9 | TB6612 PWMA | OUT | Motor A speed (PWM) |
| D10 | TB6612 PWMB | OUT | Motor B speed (PWM) |
| D11 | LCD EN | OUT | |
| D12 | LCD RS | OUT | |
| D13 | LED red | OUT | 220 Ω series |
| A0 | Photo B OUT | IN | INPUT_PULLUP, blocked = LOW |
| A1 | IR receiver OUT | IN | VS1838B |
| A2 | TB6612 BIN2 | OUT | Motor B direction |
| A3 | TB6612 BIN1 | OUT | Motor B direction |
| A4 | Photo A OUT | IN | INPUT_PULLUP, blocked = LOW |
| A5 | LED green | OUT | 220 Ω series |
| 5V | LCD VDD, TB6612 VCC, photo VCC, IR VCC | PWR | |
| GND | Common ground | PWR | Single-point ground |

### 3.2 LCD1602 Connection

| LCD Pin | Connection |
|---|---|
| VSS | GND |
| VDD | 5V |
| V0 | 10 kΩ pot wiper |
| RS | D12 |
| RW | **GND** (must be grounded, write-only mode) |
| EN | D11 |
| D0–D3 | Floating (4-bit mode) |
| D4 | D5 |
| D5 | D4 |
| D6 | D3 |
| D7 | D2 |
| A (backlight +) | 5V through 100–220 Ω |
| K (backlight −) | GND |

### 3.3 TB6612FNG Connection

| TB6612 Pin | Connection |
|---|---|
| VM | Motor supply (6–12 V recommended) |
| VCC | 5V logic |
| GND | Common ground |
| STBY | D8 |
| AIN1 | D6 |
| AIN2 | D7 |
| PWMA | D9 |
| AO1 / AO2 | Motor A |
| BIN1 | A3 |
| BIN2 | A2 |
| PWMB | D10 |
| BO1 / BO2 | Motor B |

> If Motor A/B rotation is reversed, swap AO1/AO2 or BO1/BO2. No code change needed.

### 3.4 Photo Sensors

The code uses `INPUT_PULLUP`, and **blocked = LOW**. Common slot optocouplers (ITR9608) or IR break-beam modules meet this requirement.
- Photo A: A4, carousel encoder 16 edges/rev
- Photo B: A0, card-feed exit

Wiring: VCC → 5V, GND → GND, OUT → corresponding Arduino pin. If your module outputs HIGH when blocked, add an inverter between OUT and the Arduino pin, or use a "LOW when blocked" model.

### 3.5 IR Receiver

VS1838B: VCC → 5V, GND → GND, OUT → A1. Keep the receiver away from motor wires and PWM traces.

---

## 4. Compile Switches (top of `AutoCarDealer-v2.ino`)

| Macro | Default | Description |
|---|---|---|
| `ENABLE_INFRA` | 1 | Enable Remote 1 |
| `ENABLE_INFRA2` | 1 | Enable Remote 2 |
| `ENABLE_KEYBOARD` | 0 | Enable serial commands (debug) |
| `DEBUG` | 0 | Serial debug output |
| `IR_DEBUG` | 0 | Raw IR code output |
| `ENABLE_FAST_LCD` | 0 | 0 = use `updateDisplay()`; 1 = fast partial refresh |
| `ENABLE_MISS_COMPENSATION` | 0 | 0 = slow cards can wait; 1 = 700 ms missed-card compensation (may misjudge slow cards) |
| `ENABLE_DECK_CHECK_PAUSE` | 0 | 1 = pause every deck with "Check Deck!" |

### 4.1 Key Parameters

| Macro | Recommended | Description |
|---|---|---|
| `MOTOR_A_SPEED` | 255 | Motor A full speed |
| `MOTOR_B_SPEED` | **180** | Motor B card feed; 180 tested 500+ cards with 0 double-deals; 190+ occasional |
| `CARD_IGNORE_TIME` | 80 | Post-deal blanking window |
| `PHOTO_A_DEBOUNCE` | 8 | Motor A encoder debounce |
| `PHOTO_B_DEBOUNCE` | 10 | Motor B photo debounce |
| `MOTOR_B_HARD_TIMEOUT` | 3000 | Pause if no card within 3 s |
| `MOTOR_A_TIMEOUT` | 1500 | Motor A rotation timeout |
| `MOTOR_A_STUCK_TIME` | 500 | Motor A stall detection |
| `LOOP_DELAY` | 1 | Main loop delay (ms) |

> **Speed vs double-deal test results**: 175–180 → 0 double-deals; 190 → ~1.2%; 235 → ~0.5–0.75%; 255 + strongest braking → ~0.7–1%. This is the physical sampling limit of the sensor; software cannot eliminate it. For 0 double-deals, keep 180 or enable `ENABLE_DECK_CHECK_PAUSE` for manual verification.

---

## 5. IR Remote Key Map

### 5.1 Remote 1 (`ENABLE_INFRA`)

| Key | Function | Code |
|---|---|---|
| ch- | Players −1 | `0xBA45FF00` |
| ch | Restore defaults (4 players / 3 decks / jokers) | `0xB946FF00` |
| ch+ | Players +1 | `0xB847FF00` |
| prev | Decks +1 | `0xBB44FF00` |
| next | Toggle jokers | `0xBF40FF00` |
| play | Start / pause / resume | `0xBC43FF00` |
| EQ | Stop | `0xF609FF00` |
| 0 | Remaining cards +1 | `0xE916FF00` |
| 100+ | Test Motor A | `0xE619FF00` |
| 200+ | Test Motor B | `0xF20DFF00` |
| 1 | Calibrate A | `0xF30CFF00` |
| 2 | Toggle direction | `0xE718FF00` |
| 3 | System reset | `0xA15EFF00` |
| 4 | Version info | `0xF708FF00` |
| 9 (reboot) | Soft reset | `0xB54AFF00` |
| 5 | Score X +1 | `0xE31CFF00` |
| 6 | Score X −1 | `0xA55AFF00` |
| 7 | Score Y +1 | `0xBD42FF00` |
| 8 | Score Y −1 | `0xAD52FF00` |

### 5.2 Remote 2 (`ENABLE_INFRA2`)

| Key | Function | Code |
|---|---|---|
| play | Same as IR_START | `0xFE017F80` |
| up | Same as IR_STOP | `0xED127F80` |
| + | Players +1 | `0xFA057F80` |
| - | Remaining cards +1 | `0xE41B7F80` |
| left | Decks +1 | `0xEE117F80` |
| right | Toggle jokers | `0xFC037F80` |
| down | Toggle direction | `0xF6097F80` |

> If your remote codes differ, set `IR_DEBUG=1` and open the Serial Monitor. The actual code will be printed; replace the macro definitions accordingly.

---

## 6. Operating Procedure

1. Power on: LCD shows `Homing MotorA...`; Motor A auto-homes, then the main screen appears.
2. Idle setup:
   - `ch+` / `ch-`: set players (2–8)
   - `prev`: set decks (1–3)
   - `next`: toggle jokers
   - `0`: set remaining cards (deal a fixed number per player, then stop)
   - `2`: toggle rotation direction (after toggling, press `1` to recalibrate A)
3. Press `play` to start dealing. LCD state character alternates `B`/`A`; green LED on.
4. While dealing:
   - Press `play` to pause (yellow LED); press again to resume
   - Press `EQ` to stop
   - Press 5/6/7/8 to adjust scores
5. When finished: LCD shows `All Done!`; red LED on; returns to idle.
6. Faults: `A Stuck` / `A Timeout` / `B Timeout` / `A Count Err`. Handle as prompted, then press `play` to resume or `EQ` to stop.

---

## 7. FAQ

**Q1: LCD occasionally shows garbled characters.**
A: First set `ENABLE_DECK_CHECK_PAUSE` to 0 to reduce overlap between motor start/stop and full LCD refresh. If it still happens, replace the "space-fill two lines" full refresh in `updateDisplay()` with `lcd.clear()` and add `delay(5)` before and after the full refresh. On hardware, ensure LCD RW is grounded, add 100 µF + 0.1 µF decoupling across LCD VCC-GND, and separate motor and logic supplies or use single-point grounding.

**Q2: Occasional double-deal of two cards.**
A: This is caused by Motor B stop inertia and the sensor's high-speed sampling limit, not a software bug. Keep `MOTOR_B_SPEED` at 180 (tested 500+ cards with 0 double-deals). If high speed is mandatory, enable `ENABLE_DECK_CHECK_PAUSE=1` and verify each deck manually. Pure software cannot guarantee 0 double-deals at ≥190.

**Q3: Slow card is misjudged and Motor A moves away.**
A: Set `ENABLE_MISS_COMPENSATION` to 0. Slow cards will be waited for normally; only the 3 s hard timeout will pause.

**Q4: Motor A position is offset.**
A: Press `1` to recalibrate A. If direction was toggled (`2`), recalibration is mandatory.

**Q5: IR remote does not respond.**
A: Set `IR_DEBUG=1` and check the Serial Monitor for received codes. If no code is received, check VS1838B wiring and 5 V. If codes are received but functions are wrong, replace the macro definitions with the actual codes.

**Q6: LCD contrast is wrong.**
A: Adjust the 10 kΩ potentiometer until characters are clear with no all-black or all-white screen.

---

## 8. Version Information

- Firmware display version: `Dealer v40.4`
- Hardware: Arduino Uno/Nano + TB6612FNG + LCD1602 + dual photo sensors + dual IR receivers
- Protocol: NEC infrared
- Encoding: 4-bit parallel LCD

---

## 9. License

For personal learning and non-commercial use only. Contact the author for redistribution or commercial use.
