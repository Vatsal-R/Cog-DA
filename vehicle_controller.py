"""Vehicle command transports: simulator or serial connection to a robot controller."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

VALID_COMMANDS = {"F", "B", "L", "R", "S"}
COMMAND_LABELS = {"F": "FORWARD", "B": "REVERSE", "L": "LEFT", "R": "RIGHT", "S": "STOP"}


@dataclass
class VehicleController:
    """Send `COMMAND,SPEED` packets and suppress duplicate messages.

    Packets are ASCII lines such as `F,185\\n`. Older firmware that reads the
    first command character remains safe; the provided ESP32 sketch uses speed.
    """

    port: Optional[str] = None
    baud: int = 115200
    _serial: object | None = None
    current_command: str = "S"
    current_speed: int = 0

    def connect(self) -> None:
        if not self.port:
            return
        import serial

        self._serial = serial.Serial(self.port, self.baud, timeout=0.05)
        self._serial.reset_input_buffer()
        self.send("S", force=True)

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def send(self, command: str, speed: int = 0, force: bool = False) -> bool:
        if command not in VALID_COMMANDS:
            raise ValueError(f"Unknown command: {command}")
        speed = 0 if command == "S" else max(0, min(255, int(speed)))
        if (command, speed) == (self.current_command, self.current_speed) and not force:
            return False
        if self.connected:
            self._serial.write(f"{command},{speed}\n".encode("ascii"))
            self._serial.flush()
        self.current_command, self.current_speed = command, speed
        return True

    def heartbeat(self) -> None:
        """Refresh the MCU watchdog even when gesture and speed are unchanged."""
        if self.connected:
            self._serial.write(f"{self.current_command},{self.current_speed}\n".encode("ascii"))

    def close(self) -> None:
        if self.connected:
            self.send("S", force=True)
            self._serial.close()