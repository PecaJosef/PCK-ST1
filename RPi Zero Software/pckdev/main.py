from uart import UARTHandler
import camera
import time

def main():
    #uart = UARTHandler(port="/dev/serial0", baudrate=115200) #For Raspberry Pi 5
    uart = UARTHandler(port="/dev/ttyAMA0", baudrate=115200) #For Raspberry Pi Zero 2W
    
    if camera.cameraConnected() == True:
        camera.initCamera()
    else:
        print("ERROR:Camera not connected\r\n")

    try:
        uart.send("#RPI READY")
        uart.send("$RDY") #Sends $RDY after the RPi boots up and the Python started

        while True:
            cmd = uart.read_line()
            if cmd:
                uart.process_command(cmd)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[MAIN] Exiting...")
    finally:
        uart.close()

if __name__ == "__main__":
    main()