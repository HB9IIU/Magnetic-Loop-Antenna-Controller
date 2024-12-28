import asyncio
import serial
import time
from threading import Thread
from websockets import serve
import netifaces
from serial.tools import list_ports

# Global variables for the radio state
VFOFrequency = None
Mode = None
PTTStatus = None
RFPower = None
SWR = None
InsertedCATcommand=None

# CAT Commands
CAT_COMMANDS = {
    "Information": "IF;",
    "PTTStatus": "TX;",
    "RFPower": "PC;",
    "SWR": "RM6;",  # Command to query SWR
}


def get_wifi_ip():
    """Retrieve the IP address of the Wi-Fi interface."""
    try:
        interface = "wlan0"  # Wi-Fi interface name on Raspberry Pi
        if interface in netifaces.interfaces():
            addresses = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addresses:  # Check for IPv4 address
                return addresses[netifaces.AF_INET][0]['addr']
        return None  # No IPv4 address found on wlan0
    except Exception as e:
        print(f"Error finding Wi-Fi IP: {e}")
        return None


def ModeCodeToModeName(mode_code):
    """Translate a mode code from the radio to a human-readable mode."""
    modes = {
        "1": "LSB",
        "2": "USB",
        "3": "CW-U",
        "4": "FM",
        "5": "AM",
        "6": "RTTY-L",
        "7": "CW-L",
        "8": "DATA-L",
        "9": "RTTY-U",
        "A": "DATA-FM",
        "B": "FM-N",
        "C": "DATA-U",
        "D": "AM-N",
        "E": "PSK",
        "F": "DATA-FM-N",
    }
    return modes.get(mode_code, "Unknown")

def ModeNameToModeCode(mode_name):
    """Translate a human-readable mode to the corresponding mode code."""
    modes = {
        "1": "LSB",
        "2": "USB",
        "3": "CW-U",
        "4": "FM",
        "5": "AM",
        "6": "RTTY-L",
        "7": "CW-L",
        "8": "DATA-L",
        "9": "RTTY-U",
        "A": "DATA-FM",
        "B": "FM-N",
        "C": "DATA-U",
        "D": "AM-N",
        "E": "PSK",
        "F": "DATA-FM-N",
    }
    # Invert the dictionary
    name_to_code = {v: k for k, v in modes.items()}
    return name_to_code.get(mode_name, "Unknown")
def initialize_serial_connection(port, baudrate=38400, timeout=0.05):
    """Initialize a serial connection to the radio."""
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=timeout,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
    )


def send_command(ser, command):
    """Send a command to the radio and read the response."""
    ser.write(command.encode())
    time.sleep(0.01)
    return ser.read(ser.in_waiting or 1).decode(errors="ignore")


def radio_query_thread(port):
    """Thread for querying the radio and updating global variables."""
    global VFOFrequency, Mode, PTTStatus, RFPower, SWR, InsertedCATcommand
    try:
        with initialize_serial_connection(port) as ser:
            while True:
                try:
                    if InsertedCATcommand:
                        send_command(ser, InsertedCATcommand)
                        InsertedCATcommand=None

                    # Query VFO frequency and mode
                    response = send_command(ser, CAT_COMMANDS["Information"])
                    if len(response) >= 22 and response.startswith("IF") and response.endswith(";"):
                        VFOFrequency = response[5:14]
                        Mode = ModeCodeToModeName(response[21])
                    else:
                        print(f"[WARNING] Invalid VFO response: {response}")

                    # Query PTT status
                    response = send_command(ser, CAT_COMMANDS["PTTStatus"])
                    if len(response) >= 3 and response.startswith("TX"):
                        PTTStatus = "ON" if int(response[2]) in [1, 2] else "OFF"
                    else:
                        print(f"[WARNING] Invalid PTT response: {response}")

                    # Query RF power
                    response = send_command(ser, CAT_COMMANDS["RFPower"])
                    if len(response) >= 5 and response.startswith("PC"):
                        RFPower = f"{int(response[3:5])}W"
                    else:
                        print(f"[WARNING] Invalid RF Power response: {response}")

                    # Query SWR if PTT is ON
                    if PTTStatus == "ON":
                        response = send_command(ser, CAT_COMMANDS["SWR"])
                        if len(response) >= 6 and response.startswith("RM"):
                            SWR = round(float(response[3:6]) / 100, 2)
                        else:
                            print(f"[WARNING] Invalid SWR response: {response}")
                    else:
                        SWR = None  # SWR is only relevant when transmitting

                except Exception as e:
                    print(f"[ERROR] Unexpected error in query loop: {e}")
                time.sleep(0.005)
    except serial.SerialException as e:
        print(f"[ERROR] Serial connection error: {e}")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")


# Dedicated functions for handling ESP32 commands
def handle_set_tx(argument):
    global InsertedCATcommand
    if argument==1:
        InsertedCATcommand = "TX1;"
        print ("Setting PTT to ON")
    else:
        InsertedCATcommand = "TX0;"
        print ("Setting PTT to OFF")



def handle_set_power(argument):
    global InsertedCATcommand
    InsertedCATcommand="PC"+str(argument).zfill(3)+";"
    print("Setting RF Power to: "+str(argument)+" Watts")




def handle_set_mode(argument):
    global InsertedCATcommand
    modeCode=ModeNameToModeCode(argument)
    InsertedCATcommand="MD0"+modeCode+";"
    print("Setting Mode to: "+modeCode)



def handle_set_frequ(argument):
    global InsertedCATcommand
    InsertedCATcommand = "FA" + str(argument).zfill(9) + ";"
    print("Setting frequency to: " + str(argument/1e6) + " MHz")


def other_message(message):
    print(f"Other Message: {message}")


# Function to parse and redirect incoming messages
def parse_message(message):
    try:
        if message.startswith("setTX:"):
            argument = int(message.split(":")[1])
            handle_set_tx(argument)
        elif message.startswith("setPower:"):
            argument = int(message.split(":")[1])
            handle_set_power(argument)
        elif message.startswith("setMode:"):
            argument = message.split(":")[1]
            handle_set_mode(argument)
        elif message.startswith("setFrequ:"):
            argument = int(message.split(":")[1])
            handle_set_frequ(argument)
        else:
            other_message(message)
    except Exception as e:
        print(f"Error parsing message: {message}, Error: {e}")


async def websocket_receiver(websocket):
    """Handle incoming messages from the WebSocket client."""
    try:
        async for message in websocket:
            print(f"Received from ESP32: {message}")
            parse_message(message)  # Redirect message to appropriate handler
    except Exception as e:
        print(f"[ERROR] Error receiving message: {e}")


async def websocket_sender(websocket):
    """Send updates to the WebSocket client if the state changes."""
    global VFOFrequency, Mode, PTTStatus, RFPower, SWR

    previous_state = {
        "VFOFrequency": None,
        "Mode": None,
        "PTTStatus": None,
        "RFPower": None,
        "SWR": None,
    }

    try:
        while True:
            # Compare current state with the previous state
            current_state = {
                "VFOFrequency": VFOFrequency,
                "Mode": Mode,
                "PTTStatus": PTTStatus,
                "RFPower": RFPower,
                "SWR": SWR,
            }

            if current_state != previous_state:
                await websocket.send(str(current_state))
                print(f"Sent to ESP32: {current_state}")
                previous_state = current_state.copy()

            await asyncio.sleep(0.01)  # Non-blocking wait
    except Exception as e:
        print(f"[ERROR] Error sending message: {e}")


async def websocket_handler(websocket):
    """Handle both sending and receiving messages concurrently."""
    print(f"Client connected: {websocket.remote_address}")

    try:
        await asyncio.gather(
            websocket_receiver(websocket),
            websocket_sender(websocket),
        )
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        print("Client disconnected.")


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


async def main():

    #we ca detect with OM
    """Main function to start the WebSocket server and radio query thread."""
    port = find_yaesu_cat_port()


    with initialize_serial_connection(port) as ser:
        while True:
            try:
                response = send_command(ser, "OM;")
                print(response)
            except Exception as e:
                print(e)
            time.sleep(1)

    with initialize_serial_connection(port) as ser:
        while True:

            try:
                ser.flush()
                response = send_command(ser, "FA;")
                if response.startswith("FA") and response.endswith(";"):
                    print(response)
                    frequency = int(response[2:-1] ) # Slice from index 2 (skip "FA") to the second last character (remove ";")
                    print (frequency)
            except Exception as e:
                print(e)


            time.sleep(.025)



    # Start the radio query thread
    Thread(target=radio_query_thread, args=(port,), daemon=True).start()
    wifi_ip = get_wifi_ip()
    if not wifi_ip:
        print("[ERROR] Could not determine the Wi-Fi IP address. Exiting.")
        return

    print("Local IP:", wifi_ip)
    # Start the WebSocket server
    server = await serve(websocket_handler, wifi_ip, 8765)
    print(f"WebSocket server started on ws://{wifi_ip}:8765")
    await server.wait_closed()


# Run the program
asyncio.run(main())
