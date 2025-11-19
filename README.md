# Robo-sumo3

This repository contains the `sumo_luring.ino` sketch that drives the Nano-based luring strategy robot. The code expects very specific pin assignments, so the wiring below follows the exact layout hard-coded in the sketch.

## Wiring Guide

### Power and Ground
- Use the common 6 V battery pack to power the L298N motor driver `+12V` (or `V_in`) screw terminal.
- Run the driver's `5V` output to the Arduino Nano `5V` pin so the logic shares the same supply. (If your driver does **not** expose a regulated 5 V output, instead power the Nano through `VIN` with the battery and tie grounds together.)
- Tie the Nano `GND`, the L298N `GND`, every sensor ground, and the battery negative together to keep a single ground reference.

### Motor Driver (L298N or pin-compatible)
| Function            | Nano Pin | Driver Pin |
|--------------------|----------|------------|
| Left motor forward | D5 (PWM) | IN1        |
| Left motor reverse | D6 (PWM) | IN2        |
| Right motor forward| D9 (PWM) | IN3        |
| Right motor reverse| D10 (PWM)| IN4        |
| Enable jumpers     | On       | ENA / ENB bridged so PWM arrives through IN pins |
| Motor outputs      | OUT1/OUT2 -> left motor, OUT3/OUT4 -> right motor |

### Ultrasonic Sensors (HC-SR04)
Use two identical modules mounted on the “arms.” Each has VCC, GND, TRIG, and ECHO pins.

| Sensor | VCC | GND | TRIG wire | ECHO wire |
|--------|-----|-----|-----------|-----------|
| Left   | 5 V | GND | Nano D2   | Nano D4   |
| Right  | 5 V | GND | Nano D7   | Nano D8   |

Keep the cables short and parallel to avoid cross-talk, and add a 10 kΩ series resistor on each ECHO if you want extra protection, though the Nano can read 5 V directly.

### Front IR Distance Sensor (Sharp GP2Y0A21YK0F)
- VCC → Nano `5V`
- GND → Nano `GND`
- OUT → `A0`
- Keep the three-wire JST cable away from motor leads to reduce noise. If readings jump, add a 10 µF capacitor between VCC and GND close to the sensor.

### Bottom IR Line Sensors
Connect five reflective IR modules (or discrete phototransistors) to the analog pins listed below. Power them from 5 V and GND, and route their analog outputs directly to the Nano:

| Sensor position      | Nano analog pin |
|----------------------|-----------------|
| Far left             | A1              |
| Mid-left             | A2              |
| Center               | A3              |
| Mid-right            | A4              |
| Far right            | A5              |

Calibrate each module so that black mat readings stay below the `EDGE_WHITE_THRESHOLD` (700 in the sketch) and the white border rises above it.

### Optional Debug Add-ons
- Plug a USB cable into the Nano for serial debugging (115200 baud). The code prints the active state, ultrasonic ranges, and front IR distance continuously.
- You can add an LED to show the current state by connecting it to any spare digital pin and driving it inside the state machine. This is optional and not part of the default code.

Once wired exactly as above, upload `sumo_luring.ino` to the Nano. The robot will spin in SEARCHING mode and follow the luring/presentation logic described in the sketch.

## Uploading the Sketch Without Diff Artifacts

1. Either clone this repository or open `sumo_luring.ino` in your browser and click **Raw** so you see only the sketch text.
2. Copy the contents into a new Arduino sketch folder named `sumo_luring` (the `.ino` file must share the folder name).
3. Double-check that the first line of the file starts with `// This sketch implements…` and that no lines begin with `+`, `-`, or the word `diff`.
4. Compile/upload. If you see errors like `too many decimal points in number` it means stray diff markers were pasted; delete the file contents and paste the clean sketch again.

Following the steps above ensures the IDE receives a clean file without Git diff headers (which the compiler reads as invalid C++).
