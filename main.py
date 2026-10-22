"""Real-time gesture control with variable speed and safe serial heartbeats."""

from __future__ import annotations

import argparse
import math
import time
import urllib.request
from pathlib import Path

import cv2
import mediapipe as mp

from vehicle_controller import COMMAND_LABELS, VehicleController

MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
MODEL_PATH = Path(__file__).with_name("hand_landmarker.task")
HAND_LOST_STOP_SECONDS = 0.20  # Matches the PPT heartbeat safety target.
HEARTBEAT_SECONDS = 0.10


def classify_gesture(landmarks, width: int, height: int) -> str | None:
    """Recognise five safe commands from an index-pointing hand or closed fist."""
    points = [(p.x * width, p.y * height) for p in landmarks]
    wrist = points[0]
    index_tip, index_pip = points[8], points[6]
    middle_tip, middle_pip = points[12], points[10]
    ring_tip, ring_pip = points[16], points[14]
    pinky_tip, pinky_pip = points[20], points[18]

    def extended(tip, pip) -> bool:
        return math.dist(tip, wrist) > math.dist(pip, wrist) * 1.23

    index_up = extended(index_tip, index_pip)
    others_curled = not extended(middle_tip, middle_pip) and not extended(ring_tip, ring_pip) and not extended(pinky_tip, pinky_pip)
    if not index_up and others_curled:
        return "S"
    if not (index_up and others_curled):
        return None

    dx, dy = index_tip[0] - index_pip[0], index_tip[1] - index_pip[1]
    if abs(dx) > abs(dy):
        return "R" if dx > 0 else "L"
    return "B" if dy > 0 else "F"


def depth_speed(landmarks, width: int, height: int, minimum: int) -> tuple[int, float]:
    """Map apparent palm size to PWM speed.

    Palm width grows as a hand approaches the camera, giving a stable practical
    depth proxy across cameras. Returns the PWM value and palm width in pixels.
    """
    palm_width = math.dist(
        (landmarks[5].x * width, landmarks[5].y * height),
        (landmarks[17].x * width, landmarks[17].y * height),
    )
    near, far = 230.0, 65.0
    closeness = max(0.0, min(1.0, (palm_width - far) / (near - far)))
    return round(minimum + closeness * (255 - minimum)), palm_width


def draw_status(frame, command: str, speed: int, connected: bool, palm_width: float, latency_ms: float) -> None:
    label = COMMAND_LABELS[command]
    colour = (0, 220, 0) if command != "S" else (0, 0, 230)
    cv2.rectangle(frame, (10, 10), (465, 140), (25, 25, 25), -1)
    cv2.putText(frame, f"Vehicle: {label}", (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.78, colour, 2)
    cv2.putText(frame, f"PWM speed: {speed}/255  |  depth proxy: {palm_width:.0f}px", (25, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (235, 235, 235), 1)
    mode = "ESP32/ARDUINO CONNECTED" if connected else "SIMULATOR MODE (no vehicle commands sent)"
    cv2.putText(frame, mode, (25, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (235, 235, 235), 1)
    cv2.putText(frame, f"Vision inference: {latency_ms:.1f} ms", (25, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 220, 255), 1)
    cv2.putText(frame, "Point: up/down/left/right | Fist: stop | S: emergency stop | Q: quit", (10, frame.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1)


def run(camera: int, port: str | None, baud: int, minimum_speed: int) -> None:
    controller = VehicleController(port=port, baud=baud)
    controller.connect()
    cap = cv2.VideoCapture(camera, cv2.CAP_DSHOW)
    if not cap.isOpened():
        controller.close()
        raise RuntimeError(f"Cannot open camera {camera}. Try --camera 1.")

    if not MODEL_PATH.exists():
        print("Downloading the MediaPipe hand model (first run only)...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    options = mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(MODEL_PATH)),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.65,
        min_tracking_confidence=0.65,
    )
    hands = mp.tasks.vision.HandLandmarker.create_from_options(options)
    last_hand_seen = time.monotonic()
    last_heartbeat = 0.0
    command, speed, palm_width, latency_ms = "S", 0, 0.0, 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            started = time.perf_counter()
            result = hands.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), int(time.monotonic() * 1000))
            latency_ms = (time.perf_counter() - started) * 1000
            detected = None
            if result.hand_landmarks:
                landmarks = result.hand_landmarks[0]
                for point in landmarks:
                    cv2.circle(frame, (int(point.x * frame.shape[1]), int(point.y * frame.shape[0])), 3, (0, 255, 0), -1)
                detected = classify_gesture(landmarks, frame.shape[1], frame.shape[0])
                palm_speed, palm_width = depth_speed(landmarks, frame.shape[1], frame.shape[0], minimum_speed)
                last_hand_seen = time.monotonic()
                if detected:
                    command = detected
                    speed = 0 if command == "S" else palm_speed

            if time.monotonic() - last_hand_seen > HAND_LOST_STOP_SECONDS:
                command, speed = "S", 0
            controller.send(command, speed)
            if time.monotonic() - last_heartbeat >= HEARTBEAT_SECONDS:
                controller.heartbeat()
                last_heartbeat = time.monotonic()
            draw_status(frame, controller.current_command, controller.current_speed, controller.connected, palm_width, latency_ms)
            cv2.imshow("Gesture-Controlled Vehicle", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                command, speed = "S", 0
                controller.send(command, speed, force=True)
    finally:
        hands.close()
        cap.release()
        cv2.destroyAllWindows()
        controller.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Gesture-controlled robot vehicle")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--port", help="Serial port, e.g. COM3; omit for safe simulator mode")
    parser.add_argument("--baud", type=int, default=115200, help="Controller baud rate (ESP32 default: 115200)")
    parser.add_argument("--min-speed", type=int, default=110, choices=range(0, 256), metavar="0..255", help="PWM speed for a far-away hand")
    args = parser.parse_args()
    run(args.camera, args.port, args.baud, args.min_speed)