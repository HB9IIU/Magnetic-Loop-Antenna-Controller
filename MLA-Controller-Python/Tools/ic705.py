import sys
import serial
import time
import serial.tools.list_ports
# List all available serial ports
ports = serial.tools.list_ports.comports()

# Print information about each port
for port in ports:
    print(f"Port: {port.device}, Description: {port.description}, HWID: {port.hwid}")

def connect_to_IC705():
    ports = serial.tools.list_ports.comports()
    icom_port = ""
    for port, desc, hwid in sorted(ports):
        print(port, desc)
        if "IC-705" in desc:
            icom_port = port
            break
    if icom_port != "":
        print("Found IC705 on port:", icom_port)
        ic705 = serial.Serial(icom_port, baudrate=19200)
        # return check_communication_with_IC705()
        return ic705


    else:
        print("IC 705 not found (returning False)")
        return False


def check_communication_with_IC705():
    print("Checking if IC705 is responding")
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    command = b'\x19\x00'  # Read the transceiver ID
    # print (PCT + command + EM)
    ic705.write(PCT + command + EM)
    # Radio to controller message with data
    # Reply = Preamble	Controller Address	Transceiver Address	Command number	Sub Command Number	Data Area	End of Message
    #        0xFE 0xFE	      0xE0	             0xa4	            Cn	             Sc	            Data	         0xFD
    #          1   2            3                  4                5                6                7               8

    # ic705.read_until(expected=b'\xfd')
    reply = ic705.read_until(expected=b'\xfd')
    # print("Reply:", reply, len(reply))
    returned_data = reply[6]
    if returned_data == int.from_bytes(CI_V, byteorder='big'):
        print("Bingo, IC705 is are online!")
        return True
    else:
        print("Something not ok here!!!!")
        return False


def switch_IC705_ON():
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    command = b'\x18\x01'  # POWER ON
    # print (PCT + command + EM)
    ic705.write(PCT + command + EM)


def switch_IC705_OFF():
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    command = b'\x18\x00'  # POWER OFF
    # print (PCT + command + EM)
    ic705.write(PCT + command + EM)


def get_IC705_frequency():
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    command = b'\x03'  # Get Frequency
    frequ = 0
    while frequ == 0:
        try:
            ic705.write(PCT + command + EM)
            reply = ic705.read_until(expected=b'\xfd')
            # print("Reply:", reply, len(reply))

            bloc_1 = reply[5]
            bloc_2 = reply[6]
            bloc_3 = reply[7]
            bloc_4 = reply[8]
            bloc_5 = reply[9]

            a = (bloc_1 & 0xF0) >> 4
            b = bloc_1 & 0x0F
            c = (bloc_2 & 0xF0) >> 4
            d = bloc_2 & 0x0F
            e = (bloc_3 & 0xF0) >> 4
            f = bloc_3 & 0x0F
            g = (bloc_4 & 0xF0) >> 4
            h = bloc_4 & 0x0F
            i = (bloc_5 & 0xF0) >> 4
            j = bloc_5 & 0x0F
            # print(i, j, g, h, e, f, c, d, a, b)
            frequ = int(i * 1e9 + j * 1e8 + g * 1e7 + h * 1e6 + e * 1e5 + f * 1e4 + c * 1e3 + d * 1e2 + a * 10 + b)
        except:
            print("error")
            time.sleep(.1)
    return frequ


def PTT_ON():
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    command = b'\x1C\x00\x01'  # POWER ON
    # print (PCT + command + EM)
    ic705.write(PCT + command + EM)
    # FE FE A4 E0 1C 00 01 FD


def PTT_OFF():
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    command = b'\x1C\x00\x00'  # POWER ON
    # print (PCT + command + EM)
    ic705.write(PCT + command + EM)


def get_SWR_old():
    def map_float(value, in_min, in_max, out_min, out_max):
        # Linear interpolation formula
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def calculate_swr(SWR_raw):
        if SWR_raw <= 48:
            SWR = map_float(SWR_raw, 0, 48, 1.0, 1.5)
        elif SWR_raw <= 80:
            SWR = map_float(SWR_raw, 49, 80, 1.5, 2.0)
        elif SWR_raw <= 120:
            SWR = map_float(SWR_raw, 81, 120, 2.0, 3.0)
        elif SWR_raw <= 155:
            SWR = map_float(SWR_raw, 121, 155, 3.0, 4.0)
        elif SWR_raw <= 175:
            SWR = map_float(SWR_raw, 156, 175, 4.0, 5.0)
        elif SWR_raw <= 225:
            SWR = map_float(SWR_raw, 176, 225, 5.0, 10.0)
        else:
            SWR = map_float(SWR_raw, 226, 255, 10.0, 50.0)
        return SWR

    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message

    command = b'\x15\x11'  # power
    command = b'\x15\x12'  # swr
    #print(PCT + command + EM)
    ic705.flush()
    time.sleep(.1)
    print("writing")
    ic705.write(PCT + command + EM)
    ic705.read_until(expected=b'\xfd')
    reply = ic705.read_until(expected=b'\xfd')
    ic705.flush()
    #print("Reply:", reply, len(reply))
    try:
        position_of_relevant_byte = len(reply) - 3
        bloc1 = reply[6:8]
        print(bloc1)
        int_val = int.from_bytes(bloc1, "big")
        print(int_val)
        #SWR = calculate_swr(int_val)
        #print(SWR)
        #return SWR
    except:
        pass
    # relevant_byte=bloc1
    # y = 5E-07x3 - 4E-06x2 + 0.0094x + 1
    # SWR =5e-7*relevant_byte**3-4e-6*relevant_byte**2+0.0094*relevant_byte+1
    # print (relevant_byte,SWR)
    '''
    val3 = SWR_raw;
  if (SWR_raw <= 48) {
    SWR = mapFloat(SWR_raw, 0, 48, 1.0, 1.5);
  } else if (SWR_raw <= 80) {
    SWR = mapFloat(SWR_raw, 49, 80, 1.5, 2.0);
  } else if (SWR_raw <= 120) {
    SWR = mapFloat(SWR_raw, 81, 120, 2.0, 3.0);
  } else if (SWR_raw <= 155) {
    SWR = mapFloat(SWR_raw, 121, 155, 3.0, 4.0);
  } else if (SWR_raw <= 175) {
    SWR = mapFloat(SWR_raw, 156, 175, 4.0, 5.0);
  } else if (SWR_raw <= 225) {
    SWR = mapFloat(SWR_raw, 176, 225, 5.0, 10.0);
  } else {
    SWR = mapFloat(SWR_raw, 226, 255, 10.0, 50.0);
  }

    '''

def readRFpower():
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    command = b'\x14\x0A'  # power
    # print(PCT + command + EM)
    ic705.flush()
    ic705.write(PCT + command + EM)
    reply = ic705.read_until(expected=b'\xfd')
    command_bloc = reply[4]
    print (reply)
    print (command_bloc)
    if command_bloc==b'\x14':
        bloc1 = reply[6:8]
        print(bloc1)
        int_val = int.from_bytes(bloc1, "big")
        print("Reply:", reply, len(reply), int_val)

def get_SWR():
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message

    command = b'\x15\x11'  # power
    command = b'\x15\x12'  # swr
    #print(PCT + command + EM)
    ic705.flush()
    time.sleep(.1)
    ic705.write(PCT + command + EM)
    #ic705.read_until(expected=b'\xfd')
    reply = ic705.read_until(expected=b'\xfd')

    print("Reply:", reply, len(reply))
    try:
        position_of_relevant_byte = len(reply) - 3
        bloc1 = reply[6:8]
        print(bloc1)
        int_val = int.from_bytes(bloc1, "big")
        print(int_val)
        #SWR = calculate_swr(int_val)
        #print(SWR)
        #return SWR
    except:
        pass
    # relevant_byte=bloc1
    # y = 5E-07x3 - 4E-06x2 + 0.0094x + 1
    # SWR =5e-7*relevant_byte**3-4e-6*relevant_byte**2+0.0094*relevant_byte+1
    # print (relevant_byte,SWR)
    '''
    val3 = SWR_raw;
  if (SWR_raw <= 48) {
    SWR = mapFloat(SWR_raw, 0, 48, 1.0, 1.5);
  } else if (SWR_raw <= 80) {
    SWR = mapFloat(SWR_raw, 49, 80, 1.5, 2.0);
  } else if (SWR_raw <= 120) {
    SWR = mapFloat(SWR_raw, 81, 120, 2.0, 3.0);
  } else if (SWR_raw <= 155) {
    SWR = mapFloat(SWR_raw, 121, 155, 3.0, 4.0);
  } else if (SWR_raw <= 175) {
    SWR = mapFloat(SWR_raw, 156, 175, 4.0, 5.0);
  } else if (SWR_raw <= 225) {
    SWR = mapFloat(SWR_raw, 176, 225, 5.0, 10.0);
  } else {
    SWR = mapFloat(SWR_raw, 226, 255, 10.0, 50.0);
  }

    '''


def set_FM():
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    command = b'\x01\x05\x01'  # POWER ON
    # char request[] = { 0xFE, 0xFE, IC705_CI_V_ADDRESS, 0xE0, 0x01, mode_byte, filter_byte,0xFD };
    '''
        Operatingg Mode
        00: LSB
        01: USB
        02: AM
        03: CW
        04: RTTY 
        05: FM
        06: WFM
        07: CW - R
        8: RTTY - R
        17: DV —

        Filter Setting

        01: FIL1
        02: FIL2
        03: FIL3

        '''
    # print(PCT + command + EM)
    ic705.flush()
    time.sleep(.1)
    ic705.write(PCT + command + EM)


def set_USB():
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    command = b'\x01\x01\x01'  # POWER ON
    # char request[] = { 0xFE, 0xFE, IC705_CI_V_ADDRESS, 0xE0, 0x01, mode_byte, filter_byte,0xFD };
    '''
        Operatingg Mode
        00: LSB
        01: USB
        02: AM
        03: CW
        04: RTTY 
        05: FM
        06: WFM
        07: CW - R
        8: RTTY - R
        17: DV —

        Filter Setting

        01: FIL1
        02: FIL2
        03: FIL3

        '''
    # print(PCT + command + EM)
    ic705.flush()
    time.sleep(.1)
    ic705.write(PCT + command + EM)


def set_Mode_and_Filter(mode_byte, filter_byte):
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    CMD = b'\x01'
    message = PCT + CMD + bytes([mode_byte]) + bytes([filter_byte]) + EM

    # print(PCT + command + EM)
    ic705.flush()
    time.sleep(.1)
    ic705.write(message)


def get_MODE_and_Filter():
    print("Getting Mode and Filter")
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    # request[] = {0xFE, 0xFE, IC705_CI_V_ADDRESS, 0xE0, 0x04, 0xFD};
    command = b'\x04'
    # print(PCT + command + EM)
    ic705.flush()
    time.sleep(.1)
    # print("writing")
    ic705.write(PCT + command + EM)
    reply = ic705.read_until(expected=b'\xfd')
    ic705.flush()
    # print("Reply:", reply, len(reply))
    try:
        # Define dictionaries for mode and filter mappings
        mode_mapping = {
            0x00: "LSB",
            0x01: "USB",
            0x02: "AM",
            0x03: "CW",
            0x04: "RTTY",
            0x05: "FM",
            0x06: "WFM",
            0x07: "CW - R",
            0x08: "RTTY - R",
            0x17: "DV"
        }
        filter_mapping = {
            0x01: "FIL1",
            0x02: "FIL2",
            0x03: "FIL3"
        }
        # Extract byte 6 and 7 from the reply
        byte_6 = reply[5]
        byte_7 = reply[6]

        # Retrieve mode and filter using the mappings
        mode = mode_mapping.get(byte_6, "Unknown Mode")
        filter_setting = filter_mapping.get(byte_7, "Unknown Filter")
        # print (byte_6, byte_7,mode, filter_setting)
        return byte_6, byte_7, mode, filter_setting
    except:
        pass
    '''
    Operatingg Mode
    00: LSB
    01: USB
    02: AM
    03: CW
    04: RTTY 
    05: FM
    06: WFM
    07: CW - R
    8: RTTY - R
    17: DV —

    Filter Setting
    01: FIL1
    02: FIL2
    03: FIL3

    '''


def set_IC705_frequency(frequency):
    def generate_frequency_phrase(frequency_hz):
        frequency_hz = frequency_hz * 100
        # Convert the frequency to a 10-digit string
        freq_str = str(frequency_hz).zfill(10)

        # Extract individual bytes
        byte_5 = int(freq_str[0:2], 16)
        byte_4 = int(freq_str[2:4], 16)
        byte_3 = int(freq_str[4:6], 16)
        byte_2 = int(freq_str[6:8], 16)
        byte_1 = int(freq_str[8:10], 16)

        # Construct the phrase
        phrase = [0xFE, 0xFE, 0xa4, 0xE0, byte_1, byte_2, byte_3, byte_4, byte_5, 0x00, 0xFD]

        # Convert to bytes and then to the specified format
        encoded_phrase = ''.join([f'\\x{byte:02x}' for byte in phrase])
        byte_sequence = bytes(encoded_phrase.encode().decode('unicode-escape'), 'latin1')
        return byte_sequence

    byte_sequence = generate_frequency_phrase(frequency)
    # print(byte_sequence)
    ic705.write(byte_sequence)


def get_IC705_frequency():
    CI_V = b'\xa4'
    PCT = b'\xfe\xfe' + CI_V + b'\xe0'  # https://kb3hha.com/ArticleDetails?id=6
    EM = b'\xfd'  # end of message
    command = b'\x03'  # Get Frequency
    frequ = 0
    while frequ == 0:
        try:
            ic705.write(PCT + command + EM)
            reply = ic705.read_until(expected=b'\xfd')
            # print("Reply:", reply, len(reply))

            bloc_1 = reply[5]
            bloc_2 = reply[6]
            bloc_3 = reply[7]
            bloc_4 = reply[8]
            bloc_5 = reply[9]

            a = (bloc_1 & 0xF0) >> 4
            b = bloc_1 & 0x0F
            c = (bloc_2 & 0xF0) >> 4
            d = bloc_2 & 0x0F
            e = (bloc_3 & 0xF0) >> 4
            f = bloc_3 & 0x0F
            g = (bloc_4 & 0xF0) >> 4
            h = bloc_4 & 0x0F
            i = (bloc_5 & 0xF0) >> 4
            j = bloc_5 & 0x0F
            # print(i, j, g, h, e, f, c, d, a, b)
            frequ = int(i * 1e9 + j * 1e8 + g * 1e7 + h * 1e6 + e * 1e5 + f * 1e4 + c * 1e3 + d * 1e2 + a * 10 + b)
        except:
            print("error")
            time.sleep(.1)
    return frequ


########################################################################################################
#ic705 = serial.Serial("/dev/rfcomm0", baudrate=38400)
ic705 = serial.Serial("/dev/cu.usbmodem1433201", baudrate=38400)


check_communication_with_IC705()

print(get_IC705_frequency())
while True:
    readRFpower()
    time.sleep(1)

#set_IC705_frequency(14166456)
#print(get_IC705_frequency())

set_USB()

mode_byte, filter_byte, mode, filter_setting = get_MODE_and_Filter()
set_FM()
#PTT_ON()
x=0
while x<100:
    x=x+1
    print(x);
    time.sleep(.2)
    get_SWR()
PTT_OFF()

sys.exit()

time.sleep(1)
get_SWR()

PTT_OFF()
# print (mode_byte, filter_byte, mode, filter_setting)
set_Mode_and_Filter(mode_byte, filter_byte)
# time.sleep(.1)
#
'''

#ic705 = connect_to_IC705()
ic705 = serial.Serial("/dev/rfcomm0", baudrate=38400)
if ic705!=False:
    print(check_communication_with_IC705())
    while True:


        #print(get_IC705_frequency())
        time.sleep(.1)
        #check_communication_with_IC705()

        PTT_ON()
        time.sleep(.1)
        PTT_OFF()
        time.sleep(.1)

sys.exit()
'''