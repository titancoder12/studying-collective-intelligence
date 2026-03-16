import serial
import time
import json
from typing import Optional, Dict, Any


class ESP32Robot:
    def __init__(
        self,
        port: str = "/dev/serial0",
        baudrate: int = 115200,
        timeout: float = 1.0,
        startup_delay: float = 2.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.startup_delay = startup_delay
        self.ser: Optional[serial.Serial] = None

    def connect(self) -> None:
        if self.ser is not None and self.ser.is_open:
            return

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )

        # Give ESP32 time in case opening serial resets it
        time.sleep(self.startup_delay)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def _require_serial(self) -> serial.Serial:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Serial port is not connected. Call connect() first.")
        return self.ser

    def send_raw(self, cmd: str) -> None:
        ser = self._require_serial()
        line = cmd.strip() + "\n"
        ser.write(line.encode("utf-8"))
        ser.flush()

    def read_line(self) -> Optional[str]:
        ser = self._require_serial()
        raw = ser.readline()
        if not raw:
            return None

        line = raw.decode("utf-8", errors="ignore").strip()
        return line if line else None

    def wait_response(self, timeout: float = 5.0) -> Dict[str, Any]:
        ser = self._require_serial()
        start = time.time()

        while time.time() - start < timeout:
            if ser.in_waiting:
                line = self.read_line()
                if not line:
                    continue

                parsed = self.parse_line(line)

                if parsed.get("type") == "ack":
                    return {"ok": True, "raw": line, "parsed": parsed}

                if parsed.get("type") == "err":
                    return {"ok": False, "raw": line, "parsed": parsed}

                # Ignore scan/debug lines while waiting for ack/err
            time.sleep(0.01)

        return {
            "ok": False,
            "raw": "TIMEOUT",
            "parsed": {"type": "timeout"},
        }
    
    def command(self, cmd: str, timeout: float = 5.0) -> Dict[str, Any]:
        self.send_raw(cmd)
        return self.wait_response(timeout=timeout)

    # ===== ESP32-matching high-level functions =====

    def turn(self, angle: int, timeout: float = 10.0) -> Dict[str, Any]:
        return self.command(f"T{int(angle)}", timeout=timeout)

    def move(self, angle: int, distance_mm: int, timeout: float = 20.0) -> Dict[str, Any]:
        return self.command(f"M{int(angle)},{int(distance_mm)}", timeout=timeout)

    def stop(self, timeout: float = 3.0) -> Dict[str, Any]:
        return self.command("S", timeout=timeout)

    def brake(self, timeout: float = 3.0) -> Dict[str, Any]:
        return self.command("B", timeout=timeout)

    # ===== Sensor / stream helpers =====

    def read_sensor_lines(self, duration: float = 1.0) -> list[Dict[str, Any]]:
        ser = self._require_serial()
        results: list[Dict[str, Any]] = []
        start = time.time()

        while time.time() - start < duration:
            if ser.in_waiting:
                line = self.read_line()
                if line:
                    results.append(self.parse_line(line))
            time.sleep(0.005)

        return results

    @staticmethod
    def parse_line(line: str) -> Dict[str, Any]:
        try:
            data = json.loads(line)

            if isinstance(data, dict) and "type" in data:
                return data

            return {"type": "raw", "raw": line}

        except json.JSONDecodeError:
            return {"type": "raw", "raw": line}


def main() -> None:
    robot = ESP32Robot(port="/dev/serial0", baudrate=115200)

    try:
        robot.connect()
        print("Connected to ESP32 on /dev/serial0")

        print("\nTurn 90:")
        print(robot.turn(90))

        time.sleep(1)

        print("\nMove straight 200 mm:")
        print(robot.move(0, 200))

        time.sleep(1)

        print("\nMove at -45 degrees for 150 mm:")
        print(robot.move(-45, 150))

        time.sleep(1)

        print("\nStop:")
        print(robot.stop())

        print("\nReading sensor lines for 2 seconds:")
        sensor_data = robot.read_sensor_lines(duration=2.0)
        for item in sensor_data:
            print(item)

    finally:
        robot.close()


if __name__ == "__main__":
    main()
