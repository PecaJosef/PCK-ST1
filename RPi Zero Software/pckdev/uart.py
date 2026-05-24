import serial
import time
import cmdprocessing

class UARTHandler:
    def __init__(self, port="/dev/ttyAMA0", baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(1)  #wait for the connection

    def read_line(self):
        #Read data from the UART and return it as a string
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"[UART] Received: {line}")
                return line
        return None

    def send(self, msg):
        #Send data over UART
        self.ser.write((msg + '\r\n').encode())
        print(f"[UART] Sent: {msg}")

    def process_command(self, cmd):
        #Command parsing
        if cmd.startswith("$ECHO"):
            self.send("#ECHO")
        elif cmd.startswith("$PA:ALIGN"):
            cmdprocessing.alignmentError(self)
        elif cmd.startswith("$PA:COR"):
            cmdprocessing.centerOfRotation(self)
        elif cmd.startswith("$SHUTDOWN"):
            cmdprocessing.rpiShutdown(self)
        elif cmd.startswith("$CAPTURE"):
            parts = cmd.split(":") 
            
            if len(parts) == 3:
                try:
                    exposure = float(parts[1])
                    img_name = parts[2]
                    
                    cmdprocessing.imgCapture(self, exposure, img_name)
                except ValueError:
                    self.send("$ERR:FORMAT")
            else:
                self.send("$ERR:FORMAT")
        else:
            self.send("$ERR:UNKNOWN")
            self.send("#Command error")

    def close(self):
        if self.ser.is_open:
            self.ser.close()
            print("[UART] Connection closed")