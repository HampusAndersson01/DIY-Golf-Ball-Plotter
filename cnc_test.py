import serial
import time

PORT = "COM12"       # Change this to your Arduino port
BAUD = 115200


class CNCTestController:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=2)
        time.sleep(2)  # Let Arduino reset after serial connection

        self.flush_startup_messages()

    def flush_startup_messages(self):
        while self.ser.in_waiting:
            print(self.ser.readline().decode(errors="ignore").strip())

    def send(self, command: str):
        print(f"> {command}")
        self.ser.write((command + "\n").encode())

        response = self.ser.readline().decode(errors="ignore").strip()
        print(f"< {response}")
        return response

    def enable(self):
        return self.send("ENABLE")

    def disable(self):
        return self.send("DISABLE")

    def move(self, steps: int):
        return self.send(f"MOVE {steps}")

    def speed(self, delay_microseconds: int):
        return self.send(f"SPEED {delay_microseconds}")

    def servo(self, angle: int):
        return self.send(f"SERVO {angle}")

    def close(self):
        self.ser.close()


def main():
    cnc = CNCTestController(PORT, BAUD)

    try:
        cnc.enable()

        # Slow safe speed
        cnc.speed(1200)

        # Servo test
        cnc.servo(30)
        time.sleep(0.5)

        cnc.servo(90)
        time.sleep(0.5)

        cnc.servo(120)
        time.sleep(0.5)

        # Stepper test
        cnc.move(200)
        time.sleep(0.5)

        cnc.move(-200)
        time.sleep(0.5)

        # Simple repeated motion test
        for _ in range(3):
            cnc.move(100)
            time.sleep(0.2)
            cnc.move(-100)
            time.sleep(0.2)

        cnc.servo(90)

    finally:
        cnc.disable()
        cnc.close()


if __name__ == "__main__":
    main()