
import asyncio
import serial
import time
from threading import Thread
from websockets import serve
import netifaces


# Declare global variables
VFOFrequency = None
Mode = None
PTTStatus = None
RFPower = None
SWR = None

# CAT Commands
CAT_COMMANDS = {
    "VFOFrequency": "IF;",
    "PTTStatus": "TX;",
    "RFPower": "PC;",
    "SWR": "RM6;",  # Command to query SWR
}

def get_wifi_ip():
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



def parse_mode(mode_code):
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

def initialize_serial_connection(port, baudrate=38400, timeout=0.05):
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=timeout,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
    )

def send_command(ser, command):
    ser.write(command.encode())
    time.sleep(0.01)
    return ser.read(ser.in_waiting or 1).decode(errors="ignore")

def radio_query_thread(port):
    """Thread for querying the radio and updating global variables."""
    global VFOFrequency, Mode, PTTStatus, RFPower, SWR
    try:
        with initialize_serial_connection(port) as ser:
            while True:
                try:
                    # Query VFO frequency and mode
                    response = send_command(ser, CAT_COMMANDS["VFOFrequency"])
                    if len(response) >= 22 and response.startswith("IF") and response.endswith(";"):
                        VFOFrequency = response[5:14]
                        Mode = parse_mode(response[21])
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

                time.sleep(0.001)
    except serial.SerialException as e:
        print(f"[ERROR] Serial connection error: {e}")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")

async def websocket_handler(websocket):
    """Handle WebSocket communication with a client."""
    global VFOFrequency, Mode, PTTStatus, RFPower, SWR

    # Store previous values to detect changes
    previous_state = {
        "VFOFrequency": None,
        "Mode": None,
        "PTTStatus": None,
        "RFPower": None,
        "SWR": None,
    }

    print(f"Client connected: {websocket.remote_address}")

    try:
        while True:
            # Compare global values with previous state
            current_state = {
                "VFOFrequency": VFOFrequency,
                "Mode": Mode,
                "PTTStatus": PTTStatus,
                "RFPower": RFPower,
                "SWR": SWR,
            }

            # Check for differences
            if current_state != previous_state:
                await websocket.send(str(current_state))
                print(f"Sent to ESP32: {current_state}")
                previous_state = current_state.copy()

            await asyncio.sleep(0.01)  # Prevent tight loop
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        print("Client disconnected.")

async def main():
    port = "/dev/ttyUSB0"
    # Start the radio query thread
    Thread(target=radio_query_thread, args=(port,), daemon=True).start()
    wifi_ip = get_wifi_ip()
    print("Local IP:", wifi_ip)
    # Start the WebSocket server
    server = await serve(websocket_handler, wifi_ip, 8765)
    print("WebSocket server started on ws://192.168.4.10:8765")
    await server.wait_closed()

    await server.wait_closed()

asyncio.run(main())
