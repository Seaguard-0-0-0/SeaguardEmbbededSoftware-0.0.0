#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t ch_us[8];
    bool valid[8];
    int64_t last_update_ms[8];
} rc_input_t;

extern rc_input_t g_rc;

void rc_input_init(void);
void rc_input_start(void);
void rc_input_get_snapshot(rc_input_t *out);

#endif // RC_INPUT_H


/*
Here’s the **connection configuration** (wiring) and the **expected behavior** for your setup: **FS-i6 → FS-iA6B → ESP32 → 2× ESC → 2× U01 motors**.

## 1) Wiring configuration

### A) Receiver to ESP32 (inputs)

You will read **PWM signals** from the FS-iA6B channels:

* **CH1 (Steering)** → ESP32 **RC_CH1_GPIO**
* **CH2 (Throttle)** → ESP32 **RC_CH2_GPIO**
* **CH5 (Enable/Kill switch)** → ESP32 **RC_CH5_GPIO** (use a switch on the FS-i6)

**Ground is mandatory**

* FS-iA6B **GND** → ESP32 **GND**

**Signal voltage warning (important)**

* Many receivers output **~5V PWM**. ESP32 GPIO is **3.3V max**.
* Use **level shifting** or a **resistor divider** on each PWM line (CH1/CH2/CH5) before ESP32.

### B) ESP32 to ESCs (outputs)

ESP32 generates **servo PWM (50Hz)** to drive ESC signal inputs:

* ESP32 **MOTOR_LEFT_GPIO** → **Left ESC signal**
* ESP32 **MOTOR_RIGHT_GPIO** → **Right ESC signal**

**ESC ground must be shared**

* ESC **GND** → ESP32 **GND** (common ground with receiver too)

### C) Power (recommended)

* Power the **FS-iA6B receiver** from a stable **5V** source (often from ESC BEC if available).
* Power the **ESP32** from **5V (VIN/5V pin)** or a good regulator.
* Power the **ESCs + motors** from the correct battery supply.

> Key rule: **All grounds together** (Battery/ESC GND, ESP32 GND, Receiver GND).

---

## 2) How the signals flow

**Receiver → ESP32**: ESP32 measures CH1/CH2/CH5 pulse widths (1000–2000 µs).
**ESP32 → ESCs**: ESP32 outputs PWM pulses to ESCs (also 1000–2000 µs).
So the ESP32 acts like a “gate + mixer”.

---

## 3) Expected behavior (what will happen)

### A) Normal RC control enabled

If **CH5 switch = ON** (example: pulse > 1600 µs):

* ESP32 allows RC control.
* CH2 controls **forward/back throttle**
* CH1 controls **turning**
* ESP32 mixes them into:

  * **Left motor PWM**
  * **Right motor PWM**
    This gives **differential thrust steering** (no rudder needed).

Example:

* Stick forward (CH2 high) → both motors push forward
* Stick left (CH1 low) → right motor speeds up / left motor slows down (turn left)
* Stick right → left motor speeds up / right motor slows down

### B) Kill switch / RC disabled

If **CH5 switch = OFF** (pulse < 1400 µs):

* ESP32 immediately sends **STOP** to both ESCs (neutral PWM, usually 1500 µs).
* Motors stop even if you move the sticks.

### C) Fail-safe if receiver signal is lost

If ESP32 stops receiving valid pulses (e.g., receiver off, wire loose) for > ~200 ms:

* ESP32 forces **STOP** on both motors.
  This prevents runaway.

### D) Optional “ESP override”

If you later add “ESP override” (like MQTT/autonomous):

* ESP can ignore RC even if CH5 is ON, and take control itself.
* But you should still keep CH5 as the **hard kill**.

---

## 4) What each GPIO does (simple summary)

* **RC_CH1_GPIO**: Reads steering PWM from receiver CH1
* **RC_CH2_GPIO**: Reads throttle PWM from receiver CH2
* **RC_CH5_GPIO**: Reads enable/kill switch PWM from receiver CH5
* **MOTOR_LEFT_GPIO**: Outputs PWM to Left ESC
* **MOTOR_RIGHT_GPIO**: Outputs PWM to Right ESC

---

## 5) Quick calibration notes (very important)

* Many ESCs use:

  * **1500 µs = stop/neutral**
  * **2000 µs = full forward**
  * **1000 µs = full reverse**
* Some ESCs are **forward-only** (then reverse won’t work).
* ESCs usually need **calibration** once (min/max/neutral). Do this carefully.

---

If you tell me:

1. which ESC model you use (or if U01 has built-in ESC), and
2. whether you want **reverse**,
   I’ll map the exact PWM ranges + neutral behavior to match your hardware safely.

*/