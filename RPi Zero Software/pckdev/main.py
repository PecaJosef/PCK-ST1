from uart import UARTHandler
import time

def main():
    uart = UARTHandler(port="/dev/serial0", baudrate=115200)

    try:
        while True:
            cmd = uart.read_line()
            if cmd:
                uart.process_command(cmd)
            time.sleep(0.01)  # small delay to prevent high CPU usage
    except KeyboardInterrupt:
        print("\n[MAIN] Exiting...")
    finally:
        uart.close()

if __name__ == "__main__":
    main()