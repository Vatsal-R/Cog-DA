# Gesture-Based Human–Robot Interaction

This project implements the demonstrable desktop side of the presentation and has a safe simulator mode by default. It uses a webcam and MediaPipe hand landmarks to control a vehicle direction and PWM speed.

## Demo tomorrow: safe simulator mode

With the virtual environment active, run:

```powershell
python main.py
```

No motor commands are sent unless you explicitly add `--port COMx`. The display shows the recognised command, PWM speed, depth proxy, and vision inference time. Press `S`, `Q`, or `Esc` to stop/exit.

## Gesture controls

| Gesture | Result |
| --- | --- |
| Index finger points up | Forward |
| Index finger points down | Reverse |
| Index finger points left/right | Turn left/right |
| Closed fist | Stop |

Speed varies with apparent palm size: move your hand nearer the camera to increase the PWM value. This is a practical camera-depth proxy; a monocular webcam does not provide reliable absolute distance.

## Safety implemented

- A closed fist stops immediately.
- Losing the hand for 200 ms commands a software stop.
- When connected, the PC transmits a heartbeat every 100 ms.
- The ESP32 firmware stops after 200 ms without serial data.
- The ESP32 firmware polls HC-SR04 distance every 50 ms and locks motors when an obstacle is at or below 20 cm.
- PWM output ramps in small steps, preventing sudden motor voltage spikes.

## Hardware mode (ESP32)

1. Wire an ESP32, L298N, HC-SR04, and motors according to the pin constants at the top of [esp32_vehicle.ino](esp32_vehicle/esp32_vehicle.ino). **Use a common ground** between ESP32, L298N, and sensor. Do not power motors from the ESP32.
2. In Arduino IDE, select an ESP32 board and upload `esp32_vehicle/esp32_vehicle.ino`.
3. Find its COM port, then run:

   ```powershell
   python main.py --port COM3
   ```

The protocol is `F,0..255`, `B,0..255`, `L,0..255`, `R,0..255`, or `S,0`, at 115200 baud.

> Keep the vehicle lifted from the ground during first hardware tests. Check that a hand loss, USB disconnect, fist, and a nearby obstacle all stop it before driving it.

## What remains from the presentation

The code currently uses a robust rule-based landmark recogniser so the project works without a custom training dataset. The PPT’s Dense/LSTM classifier needs labelled recordings of your five gestures; add it only after collecting and validating that dataset. The on-screen `Vision inference` number measures landmark inference time, not full camera-to-wheel latency—measure the latter with hardware for the final report.

## Files

- `main.py` — vision, gesture recognition, variable speed, watchdog
- `vehicle_controller.py` — serial packet transport and heartbeat
- `esp32_vehicle/esp32_vehicle.ino` — ESP32/L298N/HC-SR04 firmware
- `arduino/arduino_vehicle.ino` — older basic Arduino example; use ESP32 firmware for the PPT configuration