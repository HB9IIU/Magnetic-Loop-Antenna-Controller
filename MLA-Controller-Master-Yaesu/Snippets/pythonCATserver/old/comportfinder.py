import serial
from serial.tools import list_ports


def find_yaesu_cat_port():
    """
    Automatically detect the USB port to which the Yaesu radio is connected for CAT control.
    Returns the port name if detected, otherwise None.
    """
    # List all available serial ports
    ports = list_ports.comports()
    baudrate = 38400
    command = "FA;\r\n"  # CAT command to request frequency

    print("Available serial ports:")
    for port in ports:
        print(f" - {port.device} ({port.description})")

    if not ports:
        print("No serial ports detected. Is the radio connected and powered on?")
        return None

    for port in ports:
        print(f"\nTesting port: {port.device}")
        try:
            # Open serial port
            with serial.Serial(port.device, baudrate, timeout=1) as ser:
                print(f"Opened port: {port.device}")

                # Send CAT command
                print(f"Sending command: {command.strip()}")
                ser.write(command.encode())

                # Read response
                response = ser.read(100)  # Read up to 100 bytes
                print(f"Response received: {response}")

                # Validate response (check if it starts with 'FA')
                if response.startswith(b'FA'):
                    # Extract frequency part
                    try:
                        frequency = response.decode().strip().split(';')[0][2:]  # Get part after 'FA'
                        print(f"Extracted frequency: {frequency}")
                        if frequency.isdigit():
                            print(f"Yaesu CAT control detected on port: {port.device}")
                            return port.device
                        else:
                            print("Response contains non-numeric frequency.")
                    except Exception as e:
                        print(f"Error parsing frequency: {e}")
                else:
                    print(f"Response from {port.device} does not match expected format.")

        except serial.SerialException as e:
            print(f"Failed to open port {port.device}: {e}")
        except UnicodeDecodeError as e:
            print(f"Failed to decode response from {port.device}: {e}")
        except Exception as e:
            print(f"Unexpected error on port {port.device}: {e}")

    print("\nNo Yaesu CAT control detected.")
    return None


# Example usage
if __name__ == "__main__":
    detected_port = find_yaesu_cat_port()
    if detected_port:
        print(f"CAT control established on: {detected_port}")
    else:
        print("Failed to detect CAT control port.")
