import serial
import time

class UARTHandler:
    def __init__(self, port="/dev/serial0", baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(1)  # Give some time for the connection to settle
        print(f"[UART] Connected to {port} at {baudrate} baud")

    def read_line(self):
        """Reads a line from UART and returns it as a string."""
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"[UART] Received: {line}")
                return line
        return None

    def send(self, msg):
        """Sends a string over UART."""
        self.ser.write((msg + '\r\n').encode())
        print(f"[UART] Sent: {msg}")

    def process_command(self, cmd):
        """Parse and respond to commands."""
        if cmd.startswith("$ECHO"):
            self.send("#ECHO")
        else:
            self.send("#ERR:UNKNOWN")

    def close(self):
        """Close the serial port."""
        if self.ser.is_open:
            self.ser.close()
            print("[UART] Connection closed")