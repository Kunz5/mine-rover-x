# Bill of materials

## Electronics

| Part | Role | Notes |
|---|---|---|
| ESP32 DOIT DevKit V1 (38-pin) | Controller | 240 MHz dual-core, Wi-Fi/BT unused so far |
| L298N dual H-bridge | Motor driver | Drives left and right sides as two channels |
| N20 DC gear motors x4 | Drive | Wired as two pairs, tank steering |
| HC-SR04 | Ultrasonic rangefinder | Forward obstacle detection, 2–400 cm |
| IR obstacle module | Secondary proximity | Digital, active LOW |
| Metal detector sensor module | **Primary payload** | Digital comparator output |
| SG90 micro servo | Marker-drop / head actuation | Driven through the printed rack and pinion |
| 1-channel relay module | Switched auxiliary load | |
| 18650 cells + holder | Power | |

## Pin map

Pins marked **verified** are read directly from the firmware that ran on the
physical rover. Pins marked *proposed* are assignments chosen for v3 and are
not yet confirmed against the wiring.

| Signal | GPIO | Status |
|---|---|---|
| L298N ENA (right PWM) | 27 | verified |
| L298N IN1 / IN2 | 26 / 25 | verified |
| L298N ENB (left PWM) | 5 | verified |
| L298N IN3 / IN4 | 18 / 19 | verified |
| HC-SR04 TRIG / ECHO | 17 / 16 | verified |
| IR obstacle module | 4 | verified |
| Metal detector output | 34 | *proposed* — input-only pin, ADC-capable |
| Marker servo | 13 | *proposed* |

## Power notes

The L298N drops roughly 1.4–2 V across its output stage, so motor voltage sits
meaningfully below pack voltage. The ESP32 must not be powered from the L298N's
5 V regulator while the motors are under load — the rail sags on current spikes
and the ESP32 browns out mid-command. Separate the logic and motor supplies and
tie the grounds.

GPIO 34 is input-only and has no internal pull-up, so the detector module must
drive it actively or an external pull-down is required.
