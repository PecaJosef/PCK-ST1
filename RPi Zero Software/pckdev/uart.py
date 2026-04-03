import serial
import time
import cmdprocessing

class UARTHandler:
    def __init__(self, port="/dev/ttyAMA0", baudrate=115200, timeout=1):
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
        elif cmd.startswith("$PA:ALIGN"):
            cmdprocessing.alignmentError(self)
        elif cmd.startswith("$PA:COR"):
            cmdprocessing.centerOfRotation(self)
        elif cmd.startswith("$SHUTDOWN"):
            cmdprocessing.rpiShutdown(self)
        elif cmd.startswith("$CAPTURE"):
            try:
                parts = cmd.split(":", 1)[1].rsplit(".", 1)

                if len(parts) == 2:
                    exposure = float(parts[0])
                    img_name = parts[1]
                    
                    cmdprocessing.imgCapture(exposure, img_name)
                else:
                    self.send("$ERR:MALFORMED_CAPTURE")
                    
            except (ValueError, IndexError):
                self.send("$ERR:INVALID_ARGS")
            else:
                self.send("$ERR:UNKNOWN")

    def close(self):
        """Close the serial port."""
        if self.ser.is_open:
            self.ser.close()
            print("[UART] Connection closed")