# Firmware post-mortem: what was wrong with v2

v2 is the firmware that actually drove the rover. It worked well enough to demo,
which is exactly why the defects in it are worth writing down — none of them
show up on a bench, and all three of them matter the moment the rover is on
ground you care about.

The v2 source is preserved unmodified at [`firmware/v2_baseline/`](../firmware/v2_baseline/)
so the diagnosis can be checked against the real thing.

---

## Defect 1 — the failure mode was inverted

```c
long duration = pulseIn(echoPin, HIGH, 20000);
float distance = duration * 0.0343 / 2;   // → 0.0 when duration == 0
...
if (distance <= stopDistance) {           // 0 <= 15  is TRUE
  stopMotors();
```

`pulseIn()` returns `0` when it times out — that is, when **nothing echoes
back**, which happens when the path ahead is clear beyond the sensor's range.
v2 converts that to `0.0 cm` and compares it against the 15 cm stop threshold.

So an entirely clear path was read as *an obstacle zero centimetres away*, and
the rover braked. The one condition the sensor is meant to give you confidence
about is the one that stopped it.

**Fix (v3):** `pingOnce()` returns `MAX_RANGE_CM` on timeout, and also rejects
returns below the HC-SR04's 2 cm spec floor. "No echo" now means "nothing
there", which is both correct and the safe direction to fail in.

---

## Defect 2 — the rover could brick itself

```c
if (distance <= stopDistance) {
  stopMotors();
  Serial.println("Object detected within 15 cm! Stopping...");
  delay(100);
  return;                    // ← skips the serial read below
}
...
if (Serial.available() > 0) { /* read command */ }
```

The early `return` skips command handling entirely. Once an obstacle was inside
15 cm — or once defect 1 fired, which was most of the time — **no command could
be received at all**, including reverse. The rover could not be driven out of
the situation it had driven into. It had to be picked up.

**Fix (v3):** `serviceSerial()` is called unconditionally at the top of every
`loop()` iteration, before any state logic. There is no control path that can
skip it. Forward motion is refused while an obstacle is close; reverse and
turns stay available.

---

## Defect 3 — no deadman switch

```c
void moveForward() {
  digitalWrite(in1, HIGH); ...
  analogWrite(enA, motorSpeed);
}                              // returns; motors stay on indefinitely
```

Motion commands latch the motors on and return. Nothing turns them off except
another command. If the USB cable is pulled, the host crashes, or the serial
link glitches, the rover keeps driving until it hits something or the battery
dies.

For a platform whose entire purpose is to move slowly over ground that may
contain buried ordnance, an uncommanded runaway is the worst available
behaviour.

**Fix (v3):** every accepted command stamps `lastCommandMs`. The control tick
stops the motors if no command has arrived within `CMD_TIMEOUT_MS` (1.2 s).
Continuous motion requires a host that is still alive and still talking.

---

## Also fixed

| Issue in v2 | v3 |
|---|---|
| `analogWrite()` on ESP32 — a no-op on arduino-esp32 2.x, a shim on 3.x | LEDC PWM at 20 kHz, with a compile-time branch covering both core versions |
| Serial flooded at several kHz — `println` on every loop iteration | Fixed-rate CSV telemetry at 4 Hz, parseable by a host logger |
| Single unfiltered ultrasonic reading | Median-of-3, which rejects one spurious return per sample |
| Instant full PWM from standstill | Soft-start ramp (12 counts per 20 ms tick) — full torque from rest is what strips N20 gearboxes |
| Hard threshold at 15 cm, so the rover chattered at the boundary | Hysteresis: stop at 15 cm, resume only above 22 cm |
| No emergency stop | `ESTOP` latches; requires `CLEAR` then `ARM` to recover |
| Motors live at boot | Boots disarmed; `ARM` required before any motion |
| `v2.ino` and `ultransonic.ino` were byte-identical duplicates | Single source of truth |

---

## A bug the tests caught in v3 itself

The first version of v3 assigned the state machine directly from the command
handler:

```c
motion = m;
state  = (m == Motion::STOP) ? State::IDLE : State::DRIVING;   // wrong
```

The host test suite caught the consequence immediately: reversing away from an
obstacle overwrote `OBSTACLE_HOLD` with `DRIVING`, after which a forward command
was accepted again — while the obstacle was still 10 cm ahead. A latched state
flag had been allowed to disagree with the sensor.

v3 now separates the two. Commands set *intent* (`motion`); `updateState()`
derives *state* from the live range reading on every control tick. State is a
function of the world, not of the last thing the operator typed.

This is the argument for [`test/`](../test/) in a sentence: the bug was found in
a fraction of a second on a laptop, rather than by watching a rover drive into
something.
