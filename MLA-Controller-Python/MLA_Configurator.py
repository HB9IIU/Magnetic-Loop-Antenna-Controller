import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import skrf as rf  # Requires typing_extensions
import sys
from datetime import datetime
from serial.tools import list_ports
import os
import threading
from flask import Flask, request, jsonify, render_template
import logging
import requests
import math
import csv
import pandas as pd
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
import webbrowser  # Import the webbrowser module

# For Camera Control
# http://192.168.4.11/control?var=contrast&val=2
# http://192.168.4.11/control?var=saturation&val=2


global sweepNumber
sweepNumber = 0
global characterisationRoutineRunning
characterisationRoutineRunning = False

global full_path_characteritation_table
full_path_characteritation_table = []
global characterisation_table_40m
characterisation_table_40m = []
global characterisation_table_20m
characterisation_table_20m = []
global Characterisation_40m_Plot_title
Characterisation_40m_Plot_title = ""
global Characterisation_20m_Plot_title
Characterisation_20m_Plot_title = ""
global Characterisation_Initial_Wide_Band_Plot_title
Characterisation_Initial_Wide_Band_Plot_title = ""

global completion_message
completion_message = ""
global ws_frequency_array, ws_s11_db_array, ws_swr_array, ws_min_s11_db, ws_min_swr, ws_freq_at_min_s11, ws_freq_at_min_swr
global ns_frequency_array, ns_s11_db_array, ns_swr_array, ns_min_s11_db, ns_min_swr, ns_freq_at_min_s11, ns_freq_at_min_swr
ns_frequency_array = []

'''
This works only with NanoVNA-Q firmware
https://github.com/qrp73/NanoVNA-Q
https://github.com/qrp73/NanoVNA-Q/releases/tag/0.4.4
NanoVNA-Q is based on @edy555 code, includes improvements from @hugen79 and is targeted for NanoVNA-H hardware.
'''
# NanoVNA USB IDs
VNA_VID = 0x0483  # Example VID for STM32-based devices (NanoVNA typically uses this)
VNA_PID = 0x5740  # NanoVNA PID

# ESP32 Slave IP address
ESP32_IP_ADRESS = "192.168.4.1"  # IP address of your ESP32
# Create a session object to persist the connection
session = requests.Session()

# Path to the calibration files
base_dir = os.path.abspath(os.path.dirname(__file__))
calibration_kit_dir = os.path.join(base_dir, 'CalibrationKit')
charaterisations_dir = os.path.join(os.path.dirname(__file__), 'CharacterizationData')
# Create directories if they do not exist
os.makedirs(calibration_kit_dir, exist_ok=True)
os.makedirs(charaterisations_dir, exist_ok=True)
# Define the directory and CSV file paths
measured_40_to_20_meters_full_path = os.path.join(charaterisations_dir, 'measured_40_to_20_meters_full_path.csv')
detailed_40m_characterisation_file = os.path.join(charaterisations_dir, 'detailed_40m_characterisation.csv')
detailed_20m_characterisation_file = os.path.join(charaterisations_dir, 'detailed_20m_characterisation.csv')
predicted_40m_csv_file = os.path.join(charaterisations_dir, 'predicted_40m_csv_file.csv')
predicted_20m_csv_file = os.path.join(charaterisations_dir, 'predicted_20m_csv_file.csv')
lookupTableForESP32 = os.path.join(charaterisations_dir, 'lookupTableFoerESP32.txt')
longtime_recording = os.path.join('long_time_recording.csv')

# --------------------------------------------------
# Flask Routes
# --------------------------------------------------
app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/characteristics.html')
def characteristics():
    return render_template('characteristics.html')


@app.route('/checkCharacterisationStatus', methods=['GET'])
def check_characterisation_status():
    # Return the status of the characterization routine
    return jsonify({'characterisationRoutineRunning': characterisationRoutineRunning}), 200


@app.route('/wide_sweep_data')
def wide_sweep_data():
    data = {
        'ws_frequency_array': ws_frequency_array.tolist(),
        'ws_s11_db_array': ws_s11_db_array.tolist(),
        'ws_min_s11_db': ws_min_s11_db,
        'ws_freq_at_min_s11': ws_freq_at_min_s11,
    }
    return jsonify(data)


@app.route('/checkConnectionWithESP32')
def checkConnectionWithESP32():
    try:
        response_from_ESP32 = requests.get(f"http://{ESP32_IP_ADRESS}/health", timeout=2)
        if response_from_ESP32.text == "OK":
            response = "OK"
        else:
            response = "NOT OK"
    except:
        response = "NOT OK"
    return jsonify(response)


@app.route('/zoom_data')
def zoom_data():
    data = {
        'ns_frequency_array': ns_frequency_array.tolist(),
        'ns_s11_db_array': ns_s11_db_array.tolist(),
        'ns_swr_array': ns_swr_array.tolist(),
        'ns_min_s11_db': ns_min_s11_db,
        'ns_min_swr': ns_min_swr,
        'ns_freq_at_min_s11': ns_freq_at_min_s11,
    }
    return jsonify(data)


@app.route('/returnCurrentStepperPosition')
def return_current_stepper_position():
    return jsonify(get_current_stepper_position_from_ESP32())


@app.route('/setNewStepperPositionInSlave', methods=['POST'])
def set_new_stepper_position_in_slave():
    position = request.form.get('position')
    if (set_stepper_position_in_ESP32(int(position))) != True:
        sys.exit()

    # For testing, just return the received position
    return jsonify({'received_position': position})


@app.route('/moveSteppertoNewPosition', methods=['POST'])
def moveSteppertoNewPosition():
    new_position = int(request.form.get('position'))
    current_stepper_position = get_current_stepper_position_from_ESP32()
    steps_to_go = new_position - current_stepper_position
    move_duration = move_stepper_by_steps(steps_to_go)
    print("Move duration:", move_duration)
    return jsonify({'steps_to_go': steps_to_go, 'move_duration': move_duration}), 200


@app.route('/moveStepperBySteps', methods=['POST'])
def moveStepperBySteps():
    steps = int(request.form.get('steps', type=int))
    print(f"Moving stepper by {steps} steps")
    move_duration = move_stepper_by_steps(steps)
    print("returning move duration:", move_duration)
    return jsonify({'status': 'success', 'move_duration': move_duration}), 200


# Endpoint to start auto-characterization routine
@app.route('/start40mAutoCharaterisationRoutine', methods=['POST'])
def start40mAutoCharaterisationRoutine():
    global characterisation_table_20m, characterisation_table_40m, full_path_characteritation_table
    characterisation_table_40m = []
    characterisation_table_40m = []
    full_path_characteritation_table = []
    threading.Thread(target=Automatic_Characterisation_Thread_40m).start()
    return "ok", 200


@app.route('/start20mAutoCharaterisationRoutine', methods=['POST'])
def start20mAutoCharaterisationRoutine():
    global characterisation_table_20m, characterisation_table_40m, full_path_characteritation_table
    characterisation_table_40m = []
    characterisation_table_40m = []
    full_path_characteritation_table = []
    threading.Thread(target=Automatic_Characterisation_Thread_20m).start()
    return "ok", 200


@app.route('/startFullPathAutoCharacterisation', methods=['POST'])
def startFullPathAutoCharacterisation():
    global characterisation_table_20m, characterisation_table_40m, full_path_characteritation_table
    characterisation_table_40m = []
    characterisation_table_40m = []
    full_path_characteritation_table = []
    threading.Thread(target=Automatic_Characterisation_Thread_full_path).start()
    return "ok", 200


@app.route('/getCharacteristics', methods=['GET'])
def get_characteristics():
    # Read the CSV files
    detailed_40m = pd.read_csv(detailed_40m_characterisation_file)
    detailed_20m = pd.read_csv(detailed_20m_characterisation_file)
    predicted_40m = pd.read_csv(predicted_40m_csv_file)
    predicted_20m = pd.read_csv(predicted_20m_csv_file)

    # Prepare the data as dictionaries for JSON response, using the correct column names
    data = {
        '40m': {
            'detailed': detailed_40m[['Characterisation Frequency', 'Stepper Position']].to_dict(orient='records'),
            'predicted': predicted_40m[['Predicted Frequency', 'Stepper Position']].to_dict(orient='records')
        },
        '20m': {
            'detailed': detailed_20m[['Characterisation Frequency', 'Stepper Position']].to_dict(orient='records'),
            'predicted': predicted_20m[['Predicted Frequency', 'Stepper Position']].to_dict(orient='records')
        }
    }

    return jsonify(data)


@app.route('/manual_test_frequency', methods=['POST'])
def manual_test_frequency():
    try:
        # Extract the frequency value from the POST request
        frequency = request.form.get('frequency')
        if frequency:
            logging.info(f"Received frequency: {frequency} MHz for manual testing.")
            testing_frequency = float(frequency) * 1e6
            target_stepper_position_txt, estimated_duration_txt = set_new_position_for_current_vfo_frequency(
                testing_frequency)

            return jsonify({
                'estimated_duration': int(estimated_duration_txt),
            }), 200

        else:
            return jsonify({'message': 'No frequency provided'}), 400

    except Exception as e:
        logging.error(f"An error occurred: {str(e)}")
        return jsonify({'message': 'An error occurred while processing the request'}), 500


@app.route('/get_full_Characterisation_Data', methods=['GET'])
def get_full_Characterisation_Data():
    characterisation_table_poly = generate_polynomial_fit(full_path_characteritation_table)  # osolete
    characterisation_table_cubic_spline = generate_Cubic_Spline_fit(full_path_characteritation_table)

    # Convert NumPy types to native Python types
    safe_characterisation_table = convert_numpy_to_native(full_path_characteritation_table)
    safe_characterisation_table_poly = convert_numpy_to_native(characterisation_table_poly)  # osolete
    safe_characterisation_table_poly = convert_numpy_to_native(characterisation_table_cubic_spline)

    # print (safe_characterisation_table_poly)
    # print (characterisation_table_40m)
    # Return the data as JSON
    return jsonify({
        'characterisation_table': safe_characterisation_table,
        'characterisation_table_poly': safe_characterisation_table_poly,
        'samplePointColor': "#FF0000",  # red
        'plotTitle': Characterisation_Initial_Wide_Band_Plot_title
    })


@app.route('/get_40m_Characterisation_Data', methods=['GET'])
def get_40m_Characterisation_Data():
    characterisation_table_poly = generate_polynomial_fit(characterisation_table_40m)
    # Convert NumPy types to native Python types
    safe_characterisation_table = convert_numpy_to_native(characterisation_table_40m)
    safe_characterisation_table_poly = convert_numpy_to_native(characterisation_table_poly)
    # print (safe_characterisation_table_poly)
    # print (characterisation_table_40m)
    # Return the data as JSON
    return jsonify({
        'characterisation_table': safe_characterisation_table,
        'characterisation_table_poly': safe_characterisation_table_poly,
        'samplePointColor': "#FF0000",  # red
        'plotTitle': Characterisation_40m_Plot_title
    })


@app.route('/get_20m_Characterisation_Data', methods=['GET'])
def get_20m_Characterisation_Data():
    characterisation_table_poly = generate_polynomial_fit(characterisation_table_20m)
    # Convert NumPy types to native Python types
    safe_characterisation_table = convert_numpy_to_native(characterisation_table_20m)
    safe_characterisation_table_poly = convert_numpy_to_native(characterisation_table_poly)
    # print (safe_characterisation_table)
    # print (safe_characterisation_table_poly)
    # Return the data as JSON
    return jsonify({
        'characterisation_table': safe_characterisation_table,
        'characterisation_table_poly': safe_characterisation_table_poly,
        'samplePointColor': "#FF0000",  # red
        'plotTitle': Characterisation_20m_Plot_title

    })


@app.route('/saveAndUploadTablestoESP32', methods=['POST'])
def saveAndUploadTablestoESP32():
    result = merge_csv_files_to_csv(predicted_40m_csv_file, predicted_20m_csv_file, lookupTableForESP32, True, True)
    if result is not None:
        threading.Thread(target=uploadNewLookUpTableToESP32).start()
    return "ok", 200


@app.route('/getCompletionMessageStatus', methods=['POST'])
def getCompletionMessageStatus():
    return jsonify({
        'completion_message': completion_message
    })


# INFO FROM ESP
def check_slave_AP_connection():
    print("Trying to connect to slave via WiFi network 'HB9IIU-MLA'")
    connection_ok = False
    while not connection_ok:
        try:
            response = requests.get(f"http://{ESP32_IP_ADRESS}/health", timeout=2)
            if response.status_code == 200:
                print(f"Connection to Slave on HB9IIU-MLA: {response.text}")
                connection_ok = True
            else:
                print(f"Unexpected response from SLAVE: {response.status_code}, {response.text}")
        except requests.exceptions.RequestException as e:
            print("Connection timeout. Please make sure you are connected to the WiFi network 'HB9IIU-MLA'.")


def get_current_stepper_position_from_ESP32(retries=3, delay=2):
    """Fetches the current stepper position from the ESP32, retrying if needed."""
    for attempt in range(retries):
        try:
            # Use the session to make a GET request to fetch the current stepper position
            response = session.get(f"http://{ESP32_IP_ADRESS}/getCurrentStepperPosition")
            # Check if the request was successful
            if response.status_code == 200:
                # Parse the response text
                current_stepper_position = int(response.text)

                # Check for valid stepper position (not NaN)
                if not math.isnan(current_stepper_position):
                    return current_stepper_position
                else:
                    print(f"Received NaN for stepper position on attempt {attempt + 1}")
            else:
                print(f"Failed to fetch stepper position. Status code: {response.status_code} on attempt {attempt + 1}")

        except ValueError:
            print(f"Failed to convert response to integer on attempt {attempt + 1}. Response text: {response.text}")
        except Exception as e:
            print(f"An error occurred on attempt {attempt + 1}: {e}")

        # Wait before retrying
        time.sleep(delay)

    # If all attempts fail, return a default value or handle accordingly
    print(f"Failed to retrieve valid stepper position after {retries} attempts.")
    return -1  # Or another value that indicates an error


def get_stepper_position_for_current_vfo_frequency(lookup_frequency):
    """
    Sends a command to retrieve the stepper position corresponding to a given frequency.

    Args:
        lookup_frequency (int): The frequency for which to get the stepper position.

    Returns:
        dict: A dictionary containing the predicted and current stepper positions.
    """
    url = f"http://{ESP32_IP_ADRESS}/command"
    data = {
        'command': 'getStepperPositionForCurrentVFOfrequency',
        'argument': str(lookup_frequency)
    }

    try:
        response = session.post(url, data=data)

        if response.status_code == 200:
            predicted_position, current_position = response.text.split(',')
            return {
                'predicted_position': int(predicted_position),
                'current_position': int(current_position)
            }
        else:
            print(f"Failed to get stepper position. Status code: {response.status_code}")
            return None

    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")
        return None


def get_tuned_status_from_slave():
    """
    Sends the 'GetTunedStatusFromSlave' command to the ESP32 server to retrieve the
    current stepper position and the tuned frequency.

    Returns:
        dict: A dictionary containing the current stepper position and tuned frequency.
    """
    url = f"http://{ESP32_IP_ADRESS}/command"
    data = {'command': 'GetTunedStatusFromSlave'}
    try:
        response = session.post(url, data=data)
        if response.status_code == 200:
            current_stepper_position_txt, tuned_frequency_txt = response.text.split(',')
            return int(current_stepper_position_txt), int(tuned_frequency_txt)
        else:
            print(f"Failed to get tuned status. Status code: {response.status_code}")
            return None

    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")
        return None


def get_lookup_table_from_ESP32():
    """
    Fetches the lookup table from the ESP32 server in chunks using a persistent session
    and returns it as a list of tuples. Each tuple contains a frequency and a corresponding stepper position.
    """
    print("Downloading Lookup table stored in ESP32, Please Wait")

    # Define the base URL for the lookup table endpoint
    url = f"http://{ESP32_IP_ADRESS}/getLookupTable"

    # Initialize an empty list to store the table data
    table_data = []

    # Define chunk size and starting index
    chunk_size = 1000
    start_index = 0
    total_entries = 10000  # Define total expected entries or calculate dynamically

    while start_index < total_entries:
        # Define the query parameters for the current chunk
        params = {'start': start_index, 'count': chunk_size}

        try:
            # Use the session to send a GET request with query parameters
            response = session.get(url, params=params, timeout=5)

            # Check if the request was successful
            if response.status_code == 200:
                # print(f"Chunk retrieved: {start_index} to {start_index + chunk_size}")

                # Parse the response text into lines
                lookup_table = response.text
                table_lines = lookup_table.strip().split("\n")

                for line in table_lines:
                    # Skip empty lines
                    if not line.strip():
                        continue

                    # Split the line into frequency and stepper position
                    parts = line.split(',')
                    if len(parts) == 2:
                        try:
                            frequency = int(parts[0].strip())
                            stepper_position = int(parts[1].strip())
                            table_data.append((frequency, stepper_position))
                        except ValueError as e:
                            print(f"Error parsing line '{line}': {e}")
                    else:
                        print(f"Unexpected line format: '{line}'")

                # Update the start index for the next chunk
                start_index += chunk_size
            else:
                print(f"Failed to retrieve chunk, status code: {response.status_code}")
                break

        except requests.exceptions.Timeout:
            print(f"Request timed out for chunk starting at index {start_index}.")
            break
        except requests.exceptions.RequestException as e:
            print(f"An error occurred while making the request: {e}")
            break

    return table_data


# COMMANDS TO ESP
def set_stepper_position_in_ESP32(position):
    """
    Sets the stepper position on the ESP32 by sending a POST request.

    Args:
        position (int): The target stepper position to set.

    Returns:
        dict: A dictionary containing the status and message of the operation.
    """
    # Check if the position is valid (non-null and of type int)
    if position is not None and isinstance(position, int):
        # Define the URL for setting the stepper position
        url = f"http://{ESP32_IP_ADRESS}/setStepperPosition"
        try:
            # Send a POST request with the 'position' parameter
            response = session.post(url, data={'position': position})
            # response.text should be "Stepper position updated successfully"
            # Check if the request was successful
            if response.status_code == 200 and response.text == "Stepper position updated successfully":
                print(f"Stepper position set to {position}")
                time.sleep(.5)  # to make sure action took place
                return True
            else:
                print(f"Failed to set stepper position. Response: {response.text}")
                print({'status': 'error', 'message': f'Failed to set stepper position. Response: {response.text}'})
                sys.exit()
        except requests.exceptions.RequestException as e:
            # Handle exceptions during the request
            print(f"An error occurred: {e}")
            print({'status': 'error', 'message': f'An error occurred: {e}'})
            sys.exit()

    else:
        print('Invalid position provided')
        return {'status': 'error', 'message': 'Invalid position provided'}


def move_stepper_by_steps(number_of_steps):
    """
    Sends the 'moveBySteps' command to the ESP32 server to move the stepper motor
    by a specified number of steps.

    Args:
        number_of_steps (int): The number of steps to move the stepper motor.

    Returns:
        dict: A dictionary containing the target stepper position and the estimated duration.
    """
    url = f"http://{ESP32_IP_ADRESS}/command"
    data = {
        'command': 'moveBySteps',
        'argument': str(number_of_steps)
    }
    try:
        response = session.post(url, data=data)
        if response.status_code == 200:
            target_stepper_position_txt, estimated_duration_txt = response.text.split(',')
            estimated_duration = int(estimated_duration_txt)
            print("Moving by", number_of_steps, "steps")
            return estimated_duration
        else:
            print(f"Failed to move by steps. Status code: {response.status_code}")
            return None

    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")
        return None


def set_tuned_frequency_to_preference(lookup_frequency):
    """
    Sends a command to set the tuned frequency on the ESP32 and adjust the stepper position accordingly.

    Args:
        lookup_frequency (int): The frequency to set.

    Returns:
        dict: A dictionary containing the new stepper position and the tuned frequency.
    """
    url = f"http://{ESP32_IP_ADRESS}/command"
    data = {
        'command': 'setTunedFrequToPreferenceOnSlave',
        'argument': str(lookup_frequency)
    }

    try:
        response = session.post(url, data=data, timeout=20)  # Timeout set to 10 seconds

        if response.status_code == 200:
            position_for_given_frequency, tuned_frequency = response.text.split(',')
            return {
                'position_for_given_frequency': int(position_for_given_frequency),
                'tuned_frequency': int(tuned_frequency)
            }
        else:
            print(f"Failed to set tuned frequency. Status code: {response.status_code}")
            return None

    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")
        return None


def set_new_position_for_current_vfo_frequency(target_frequency):
    """
    Sends a command to set a new stepper position for the given frequency and estimates the movement duration.

    Args:
        target_frequency (int): The target frequency to set the new position for.

    Returns:
        dict: A dictionary containing the target stepper position and the estimated duration for the movement.
    """
    url = f"http://{ESP32_IP_ADRESS}/command"
    data = {
        'command': 'SetNewPositionForCurrentVFOfrequency',
        'argument': str(target_frequency)
    }

    try:
        response = session.post(url, data=data)

        if response.status_code == 200:
            target_stepper_position_txt, estimated_duration_txt = response.text.split(',')
            return int(target_stepper_position_txt), int(estimated_duration_txt)

        else:
            print(f"Failed to set new position. Status code: {response.status_code}")
            return None

    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")
        return None


def uploadNewLookUpTableToESP32():
    """
    Uploads a new lookup table to the ESP32 server using a persistent session.
    Reads the lookup_table.csv file, formats it, and sends it via a POST request.
    """
    global completion_message
    print("Uploding Lookup table, please wait")
    # Define the base URL for the lookup table endpoint
    url = f"http://{ESP32_IP_ADRESS}/uploadLookupTable"
    # Read the lookup_table.csv file into a DataFrame
    try:
        # Load the CSV file into a pandas DataFrame
        lookup_table = pd.read_csv(lookupTableForESP32)

        # Initialize a list to hold the table lines
        table_lines = []

        # Convert the DataFrame to a list of lines (strings)
        for index, row in lookup_table.iterrows():
            frequency = int(row['Predicted Frequency'])
            stepper_position = int(row['Stepper Position'])
            table_lines.append(f"{frequency},{stepper_position}")

        # Join all lines into a single string separated by newline characters
        table_data = "\n".join(table_lines)

        # Use the session to send the table data via a POST request
        response = session.post(url, data=table_data, timeout=20)  # Set timeout to 10 seconds

        # Check if the request was successful
        if response.status_code == 200:
            completion_message = "Lookup table successfully uploaded to ESP32 Controller"
            print(completion_message)
            time.sleep(1)
            completion_message = ""
        else:
            print(f"Failed to upload lookup table, status code: {response.status_code}")
            completion_message = "Lookup table to ESP32 Controller Failed"
            print(completion_message)
            time.sleep(1)
            completion_message = ""

    except FileNotFoundError:
        print("The file " + lookupTableForESP32 + " was not found.")
    except pd.errors.EmptyDataError:
        print(lookupTableForESP32 + " file is empty.")
    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")
        completion_message = "Lookup table to ESP32 Controller Failed. Check If Controller is ON"
        print(completion_message)
        time.sleep(1)
        completion_message = ""


# RELATED TO CHARACTERISATION
def Automatic_Characterisation_Thread_40m():
    global characterisationRoutineRunning
    global completion_message
    characterisationRoutineRunning = True
    global characterisation_table_40m
    global Characterisation_40m_Plot_title
    pos1 = stepper_pos_from_csv_data_cubic_spline(6990000)
    pos2 = stepper_pos_from_csv_data_cubic_spline(7350000)
    # pos1 = stepper_pos_from_csv_data_poly_model(6990000)  # cubic spline is much better
    # pos2 = stepper_pos_from_csv_data_poly_model(7350000)
    total_steps_pos1_to_pos2 = pos2 - pos1
    n_measures_for_range = 15
    increment = int(total_steps_pos1_to_pos2 / n_measures_for_range)
    pos = pos1
    forward_pos_list = []

    while pos < pos2:
        forward_pos_list.append(pos)
        pos = pos + increment
    backward_pos_list = forward_pos_list[::-1]  # Invert (mirror) the list
    # Add half increment to each element of forward list
    backward_pos_list = [item + int(increment / 2) for item in forward_pos_list]
    # Remove the last item
    backward_pos_list.pop()
    # Reversing the list
    backward_pos_list.reverse()
    total_samples = len(forward_pos_list) + len(backward_pos_list)

    characterisation_table_40m = []
    sample_counter = 0
    # Moving forward
    for position in forward_pos_list:
        current_stepper_position = get_current_stepper_position_from_ESP32()
        steps_to_go = position - current_stepper_position
        estimated_movement_duration = move_stepper_by_steps(steps_to_go)
        print("Moving to pos:", position, " (", steps_to_go, "steps to go)")
        time.sleep(estimated_movement_duration / 1000)  # wait for the movement to complete
        lastSweepNumber = sweepNumber
        while sweepNumber < lastSweepNumber + 1:  # to be sure to have a 'fresh' sweep
            print("Waiting for sweep to complete")
            time.sleep(1)
        current_stepper_position = get_current_stepper_position_from_ESP32()
        characterisation_table_40m.append((current_stepper_position, latest_zoomed_resonance_frequency))
        sample_counter = sample_counter + 1
        frequ_for_title = str(int(latest_zoomed_resonance_frequency / 1e3) / 1000)
        Characterisation_40m_Plot_title = "40m Characterisation Plot (Forward Mode - Sample " + str(
            sample_counter) + "/" + str(
            total_samples) + " @ " + frequ_for_title + " MHz)"  # Moving backwards
    for position in backward_pos_list:
        current_stepper_position = get_current_stepper_position_from_ESP32()
        steps_to_go = position - current_stepper_position
        estimated_movement_duration = move_stepper_by_steps(steps_to_go)
        print("Moving to pos:", position, " (", steps_to_go, "steps to go)")
        time.sleep(estimated_movement_duration / 1000)  # wait for the movement to complete
        lastSweepNumber = sweepNumber
        while sweepNumber < lastSweepNumber + 1:  # to be sure to have a 'fresh' sweep
            print("Waiting for sweep to complete")
            time.sleep(1)
        current_stepper_position = get_current_stepper_position_from_ESP32()
        characterisation_table_40m.append((current_stepper_position, latest_zoomed_resonance_frequency))
        sample_counter = sample_counter + 1
        frequ_for_title = str(int(latest_zoomed_resonance_frequency / 1e3) / 1000)
        Characterisation_40m_Plot_title = "40m Characterisation Plot (Reverse Mode - Sample " + str(
            sample_counter) + "/" + str(
            total_samples) + " @ " + frequ_for_title + " MHz)"
    Characterisation_40m_Plot_title = "40m Characterisation Plot (" + str(sample_counter) + "samples)"

    with open(detailed_40m_characterisation_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Stepper Position", "Characterisation Frequency"])  # Header
        writer.writerows(characterisation_table_40m)
    create_lookup_table_poly(detailed_40m_characterisation_file, predicted_40m_csv_file, 5000)

    characterisationRoutineRunning = False
    completion_message = ("40 meters Characterisation Completed")
    time.sleep(1)
    completion_message = ("")


def Automatic_Characterisation_Thread_20m():
    global characterisationRoutineRunning
    global completion_message
    characterisationRoutineRunning = True
    global characterisation_table_20m
    global Characterisation_20m_Plot_title

    pos1 = stepper_pos_from_csv_data_cubic_spline(13990000)  # could do poly instead
    pos2 = stepper_pos_from_csv_data_cubic_spline(14400000)
    total_steps_pos1_to_pos2 = pos2 - pos1
    n_measures_for_range = 15
    increment = int(total_steps_pos1_to_pos2 / n_measures_for_range)
    pos = pos1
    forward_pos_list = []

    # Generating list of positions
    while pos < pos2:
        forward_pos_list.append(pos)
        pos = pos + increment
    # Add half increment to each element of forward list
    backward_pos_list = [item + int(increment / 2) for item in forward_pos_list]
    # Remove the last item
    backward_pos_list.pop()
    # Reversing the list
    backward_pos_list.reverse()
    characterisation_table_20m = []
    sample_counter = 0
    total_samples = len(forward_pos_list) + len(backward_pos_list)
    # Moving forward
    for position in forward_pos_list:
        current_stepper_position = get_current_stepper_position_from_ESP32()
        steps_to_go = position - current_stepper_position
        estimated_movement_duration = move_stepper_by_steps(steps_to_go)
        print("Moving to pos:", position, " (", steps_to_go, "steps to go)")
        time.sleep(estimated_movement_duration / 1000)  # wait for the movement to complete
        lastSweepNumber = sweepNumber
        while sweepNumber < lastSweepNumber + 1:  # to be sure to have a 'fresh' sweep
            print("Waiting for sweep to complete")
            time.sleep(1)
        current_stepper_position = get_current_stepper_position_from_ESP32()
        characterisation_table_20m.append((current_stepper_position, latest_zoomed_resonance_frequency))
        sample_counter = sample_counter + 1
        frequ_for_title = str(int(latest_zoomed_resonance_frequency / 1e3) / 1000)
        Characterisation_20m_Plot_title = "20m Characterisation Plot (Forward Mode -Sample " + str(
            sample_counter) + "/" + str(
            total_samples) + " @ " + frequ_for_title + " MHz)"
    # Moving backwards
    for position in backward_pos_list:
        current_stepper_position = get_current_stepper_position_from_ESP32()
        steps_to_go = position - current_stepper_position
        estimated_movement_duration = move_stepper_by_steps(steps_to_go)
        print("Moving to pos:", position, " (", steps_to_go, "steps to go)")
        time.sleep(estimated_movement_duration / 1000)  # wait for the movement to complete
        lastSweepNumber = sweepNumber
        while sweepNumber < lastSweepNumber + 1:  # to be sure to have a 'fresh' sweep
            print("Waiting for sweep to complete")
            time.sleep(1)
        current_stepper_position = get_current_stepper_position_from_ESP32()
        characterisation_table_20m.append((current_stepper_position, latest_zoomed_resonance_frequency))
        sample_counter = sample_counter + 1
        frequ_for_title = str(int(latest_zoomed_resonance_frequency / 1e3) / 1000)
        Characterisation_20m_Plot_title = "20m Characterisation Plot (Reverse Mode - Sample " + str(
            sample_counter) + "/" + str(
            total_samples) + " @ " + frequ_for_title + " MHz)"
    Characterisation_20m_Plot_title = "20m Characterisation Plot (" + str(sample_counter) + "samples)"

    with open(detailed_20m_characterisation_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Stepper Position", "Characterisation Frequency"])  # Header
        writer.writerows(characterisation_table_20m)
    create_lookup_table_poly(detailed_20m_characterisation_file, predicted_20m_csv_file, 5000)
    characterisationRoutineRunning = False
    completion_message = ("20 meters Characterisation Completed")
    time.sleep(1)
    completion_message = ("")


def Automatic_Characterisation_Thread_full_path():
    global characterisationRoutineRunning
    global completion_message
    characterisationRoutineRunning = True
    global Characterisation_Initial_Wide_Band_Plot_title
    # we first need to move before 7 Mhz
    lastSweepNumber = sweepNumber
    while sweepNumber < lastSweepNumber + 1:  # to be sure to have a 'fresh' sweep
        print("waiting for sweep to complete")
        time.sleep(1)
    set_stepper_position_in_ESP32(999999)  # arbitrary but large enough
    while latest_zoomed_resonance_frequency > 6_900_000:
        estimated_movement_duration = move_stepper_by_steps(-20000)
        time.sleep(estimated_movement_duration / 1000)  # wait for the movement to complete
        lastSweepNumber = sweepNumber
        while sweepNumber < lastSweepNumber + 1:  # to be sure to have a 'fresh' sweep
            print("Waiting for sweep to complete")
            time.sleep(1)
    # 'Home' position < 7 Mhz reached, we can start
    global full_path_characteritation_table
    characterisation_increments = 15000
    set_stepper_position_in_ESP32(10000)  # just a bit > 0
    full_path_characteritation_table = []
    lastSweepNumber = sweepNumber
    while sweepNumber < lastSweepNumber + 1:  # to be sure to have a 'fresh' sweep
        print("waiting for sweep to complete")
        time.sleep(1)
    charaterisation_stepper_position = get_current_stepper_position_from_ESP32()
    full_path_characteritation_table.append((charaterisation_stepper_position, latest_zoomed_resonance_frequency))
    # Start the while loop
    wide_band_sweep_number = 0
    while latest_zoomed_resonance_frequency < 14350000:
        estimated_movement_duration = move_stepper_by_steps(characterisation_increments)
        time.sleep(estimated_movement_duration / 1000)  # wait for the movement to complete
        lastSweepNumber = sweepNumber
        while sweepNumber < lastSweepNumber + 1:  # to be sure to have a 'fresh' sweep
            print("Waiting for sweep to complete")
            time.sleep(1)
        charaterisation_stepper_position = get_current_stepper_position_from_ESP32()
        full_path_characteritation_table.append((charaterisation_stepper_position, latest_zoomed_resonance_frequency))
        wide_band_sweep_number = wide_band_sweep_number + 1
        frequ_for_title = int(latest_zoomed_resonance_frequency / 1e5) / 10
        Characterisation_Initial_Wide_Band_Plot_title = "Initial Wide Band Characterisation Plot (now at " + str(
            frequ_for_title) + " MHz, sweep n°" + str(wide_band_sweep_number) + ")"
    Characterisation_Initial_Wide_Band_Plot_title = "Initial Wide Band Characterisation Plot"
    with open(measured_40_to_20_meters_full_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Stepper Position", "Characterisation Frequency"])  # Header
        writer.writerows(full_path_characteritation_table)
    # returning home
    current_stepper_position = get_current_stepper_position_from_ESP32()
    steps_to_go = -(current_stepper_position - 10000)
    move_duration = move_stepper_by_steps(steps_to_go)
    print("Move duration:", move_duration)
    characterisationRoutineRunning = False
    completion_message = ("Initial Characterisation Completed")
    time.sleep(1)
    completion_message = ("")


def stepper_pos_from_csv_data_cubic_spline(frequency):
    # Load the CSV file into a Pandas DataFrame
    data = pd.read_csv(measured_40_to_20_meters_full_path)

    # Extract the stepper positions and frequencies
    X = data['Stepper Position'].values  # Stepper Position
    y = data['Characterisation Frequency'].values  # Frequency

    # Fit a Cubic Spline to the data (frequencies as the independent variable)
    cubic_spline = CubicSpline(y, X)

    # Get the stepper position corresponding to the input frequency
    stepper_position_solution = cubic_spline(frequency)

    # Convert the solution to a scalar if it's an array
    if isinstance(stepper_position_solution, np.ndarray):
        stepper_position_solution = stepper_position_solution.item()

    # Plotting
    plotting = True
    if plotting == True:
        plt.figure(figsize=(10, 6))

        # Plot the original data points (Stepper Position vs Characterisation Frequency)
        plt.scatter(X, y, color='red', label='Measured Data')

        # Create a range of frequencies for plotting the model
        y_range = np.linspace(min(y), max(y), 500)
        X_spline = cubic_spline(y_range)

        # Plot the Cubic Spline curve (Predicted Stepper Position vs Frequency)
        plt.plot(X_spline, y_range, color='blue', label='Cubic Spline Model')

        # Add labels and title
        plt.xlabel('Stepper Position')
        plt.ylabel('Characterisation Frequency')
        plt.title('Stepper Position vs Characterisation Frequency (Measured Data and Cubic Spline)')
        plt.legend()

        # Show plot
        plt.grid(True)
        plt.show()

    # Return the rounded stepper position
    return round(stepper_position_solution)


def stepper_pos_from_csv_data_poly_model(frequency):
    # Load the CSV file into a Pandas DataFrame
    data = pd.read_csv(measured_40_to_20_meters_full_path)

    # Extract the stepper positions and frequencies
    X = data['Characterisation Frequency'].values.reshape(-1, 1)  # Frequency as the independent variable
    y = data['Stepper Position'].values  # Stepper Position as the dependent variable

    # Create polynomial features (degree 3)
    poly = PolynomialFeatures(degree=3)
    X_poly = poly.fit_transform(X)

    # Fit the polynomial regression model
    poly_model = LinearRegression()
    poly_model.fit(X_poly, y)

    # Predict the stepper position for the given frequency
    frequency_poly = poly.transform([[frequency]])
    stepper_position_solution = poly_model.predict(frequency_poly)

    # Convert the solution to a scalar if it's an array
    if isinstance(stepper_position_solution, np.ndarray):
        stepper_position_solution = stepper_position_solution.item()

    # Plotting
    plotting = True
    if plotting:
        plt.figure(figsize=(10, 6))

        # Plot the original data points (Stepper Position vs Characterisation Frequency)
        plt.scatter(data['Characterisation Frequency'], data['Stepper Position'], color='red', label='Measured Data')

        # Create a range of frequencies for plotting the model
        X_range = np.linspace(min(X), max(X), 500).reshape(-1, 1)
        X_range_poly = poly.transform(X_range)
        y_range_poly = poly_model.predict(X_range_poly)

        # Plot the polynomial regression curve
        plt.plot(X_range, y_range_poly, color='blue', label='Polynomial Degree 3 Model')

        # Add labels and title
        plt.xlabel('Characterisation Frequency')
        plt.ylabel('Stepper Position')
        plt.title('Stepper Position vs Characterisation Frequency (Measured Data and Polynomial Model)')
        plt.legend()

        # Show plot
        plt.grid(True)
        plt.show()

    # Return the rounded stepper position
    return round(stepper_position_solution)


def frequency_from_stepper_pos_cubic_spline(stepper_position):
    # Load the CSV file into a Pandas DataFrame
    data = pd.read_csv(measured_40_to_20_meters_full_path)

    # Extract the stepper positions and frequencies
    X = data['Stepper Position'].values  # Stepper Position
    y = data['Characterisation Frequency'].values  # Frequency

    # Fit a Cubic Spline to the data (stepper positions as the independent variable)
    cubic_spline = CubicSpline(X, y)

    # Get the frequency corresponding to the input stepper position
    frequency_solution = cubic_spline(stepper_position)

    # Convert the solution to a scalar if it's an array
    if isinstance(frequency_solution, np.ndarray):
        frequency_solution = frequency_solution.item()

    plotting = False
    if plotting == True:
        # Plotting
        plt.figure(figsize=(10, 6))

        # Plot the original data points (Stepper Position vs Characterisation Frequency)
        plt.scatter(X, y, color='red', label='Measured Data')

        # Create a range of stepper positions for plotting the model
        X_range = np.linspace(min(X), max(X), 500)
        y_spline = cubic_spline(X_range)

        # Plot the Cubic Spline curve (Predicted Frequency vs Stepper Position)
        plt.plot(X_range, y_spline, color='blue', label='Cubic Spline Model')

        # Add labels and title
        plt.xlabel('Stepper Position')
        plt.ylabel('Characterisation Frequency')
        plt.title('Stepper Position vs Characterisation Frequency (Measured Data and Cubic Spline)')
        plt.legend()

        # Show plot
        plt.grid(True)
        plt.show()

    # Return the rounded frequency
    return round(frequency_solution)


def create_lookup_table_poly(source_file, output_file, points, degree=3):
    # Load the CSV file into a Pandas DataFrame
    data = pd.read_csv(source_file)

    # Extract the stepper positions and frequencies
    X = data['Stepper Position'].values.reshape(-1, 1)  # Feature (Stepper Position)
    y = data['Characterisation Frequency'].values  # Target (Frequency)

    # Define polynomial features
    poly = PolynomialFeatures(degree=degree)
    X_poly = poly.fit_transform(X)

    # Initialize and train the model
    model = LinearRegression()
    model.fit(X_poly, y)

    # Generate evenly distributed stepper positions between the min and max values in the data
    min_position = data['Stepper Position'].min()
    max_position = data['Stepper Position'].max()

    # Ensure stepper positions are integers by rounding and converting to int
    stepper_positions = np.linspace(min_position, max_position, points).round().astype(int).reshape(-1, 1)

    # Predict frequencies using the Polynomial Regression model
    stepper_positions_poly = poly.transform(stepper_positions)
    predicted_frequencies = model.predict(stepper_positions_poly)

    # Round the predicted frequencies to the nearest integer
    predicted_frequencies_rounded = np.round(predicted_frequencies).astype(int)

    # Create a new DataFrame with the results, ensuring stepper positions and frequencies are integers
    predicted_data = pd.DataFrame({
        'Stepper Position': stepper_positions.flatten(),
        'Predicted Frequency': predicted_frequencies_rounded
    })

    # Save the predicted data to the CSV file
    predicted_data.to_csv(output_file, index=False)

    # Plotting
    plt.figure(figsize=(10, 6))

    # Plot the original data points (Stepper Position vs Characterisation Frequency)
    plt.scatter(X, y, color='red', label='Measured Data')

    # Plot the Polynomial Regression curve (Predicted Stepper Position vs Frequency)
    plt.plot(stepper_positions, predicted_frequencies, color='blue', label=f'Polynomial Regression (degree={degree})')

    # Add labels and title
    plt.xlabel('Stepper Position')
    plt.ylabel('Characterisation Frequency')
    plt.title('Stepper Position vs Characterisation Frequency (Polynomial Regression)')
    plt.legend()

    # Show plot
    plt.grid(True)
    plt.show()

    print(f"Lookup table successfully created and saved to {output_file}")


def merge_csv_files_to_csv(filename1, filename2, output_filename, remove_duplicates=False, verbose=False):
    df1, df2 = None, None

    # Check if filename1 exists
    if os.path.exists(filename1):
        df1 = pd.read_csv(filename1)
        if verbose:
            print(f"File '{filename1}' loaded successfully.")

    # Check if filename2 exists
    if os.path.exists(filename2):
        df2 = pd.read_csv(filename2)
        if verbose:
            print(f"File '{filename2}' loaded successfully.")

    # Handle cases based on file existence
    if df1 is not None and df2 is not None:
        # Both files exist, concatenate them
        combined_df = pd.concat([df1, df2], ignore_index=True)
    elif df1 is not None:
        # Only filename1 exists
        combined_df = df1
    elif df2 is not None:
        # Only filename2 exists
        combined_df = df2
    else:
        # Neither file exists
        if verbose:
            print("Neither file exists. Returning None.")
        return None

    # Remove duplicates if specified
    if remove_duplicates:
        combined_df = combined_df.drop_duplicates()
        if verbose:
            print("Duplicate rows have been removed.")

    # Reorder and rename columns (if both files exist)
    required_columns = ['Predicted Frequency', 'Stepper Position']
    combined_df = combined_df[required_columns]

    # Save the combined DataFrame to a CSV file
    combined_df.to_csv(output_filename, index=False)

    print(f"Files processed and saved successfully into {output_filename}")
    return combined_df


def generate_polynomial_fit(characterisation_table):
    if not characterisation_table:
        return []

    # Extract the stepper positions and resonance frequencies
    stepper_positions = np.array([x[0] for x in characterisation_table]).reshape(-1, 1)
    resonance_frequencies = np.array([x[1] for x in characterisation_table])

    # Create polynomial features
    poly = PolynomialFeatures(degree=3)
    stepper_positions_poly = poly.fit_transform(stepper_positions)

    # Fit the polynomial regression model
    model = LinearRegression()
    model.fit(stepper_positions_poly, resonance_frequencies)

    # Generate a smooth set of stepper positions for the polynomial line (300 points for smoothness)
    stepper_positions_smooth = np.linspace(min(stepper_positions), max(stepper_positions), 300).reshape(-1, 1)
    stepper_positions_smooth_poly = poly.transform(stepper_positions_smooth)

    # Predict the resonance frequencies for the smooth set of stepper positions
    predicted_frequencies = model.predict(stepper_positions_smooth_poly)

    # Round the stepper positions and predicted frequencies to integers
    stepper_positions_smooth_int = stepper_positions_smooth.astype(int).flatten()
    predicted_frequencies_int = np.rint(predicted_frequencies).astype(int)

    # Return the polynomial table as a list of tuples (smooth stepper positions, predicted resonance frequencies)
    characterisation_table_poly = list(zip(stepper_positions_smooth_int, predicted_frequencies_int))
    return characterisation_table_poly


def generate_Cubic_Spline_fit(characterisation_table):
    if not characterisation_table:
        return []

    # Extract the stepper positions and resonance frequencies
    stepper_positions = np.array([x[0] for x in characterisation_table])
    resonance_frequencies = np.array([x[1] for x in characterisation_table])

    # Fit the cubic spline model
    cubic_spline = CubicSpline(stepper_positions, resonance_frequencies)

    # Generate a smooth set of stepper positions for the cubic spline line (300 points for smoothness)
    stepper_positions_smooth = np.linspace(min(stepper_positions), max(stepper_positions), 300)

    # Predict the resonance frequencies for the smooth set of stepper positions
    predicted_frequencies = cubic_spline(stepper_positions_smooth)

    # Round the stepper positions and predicted frequencies to integers
    stepper_positions_smooth_int = stepper_positions_smooth.astype(int).flatten()
    predicted_frequencies_int = np.rint(predicted_frequencies).astype(int)

    # Return the cubic spline table as a list of tuples (smooth stepper positions, predicted resonance frequencies)
    characterisation_table_cubispline = list(zip(stepper_positions_smooth_int, predicted_frequencies_int))

    return characterisation_table_cubispline


def convert_numpy_to_native(data):
    # Helper function to convert numpy types to native Python types
    if isinstance(data, np.ndarray):
        return data.tolist()
    elif isinstance(data, (np.int64, np.int32, np.float64, np.float32)):
        return data.item()
    elif isinstance(data, list):
        return [convert_numpy_to_native(i) for i in data]
    elif isinstance(data, tuple):
        return tuple(convert_numpy_to_native(i) for i in data)
    else:
        return data


# --------------------------------------------------
# Utility Functions
# --------------------------------------------------

def get_vna_port() -> str:
    """
    Automatically detect the NanoVNA serial port by scanning available ports.
    Returns the port if found, raises OSError if the NanoVNA is not detected.
    """
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == VNA_VID and device.pid == VNA_PID:
            print(f"Found NanoVNA on port: {device.device}")
            return device.device
    raise OSError("NanoVNA device not found")


def initialize_serial(port: str, baudrate=115200, timeout=5):
    """
    Initialize the serial connection with the NanoVNA.
    Args:
    - port: The serial port for the NanoVNA.
    - baudrate: Baud rate for the serial connection (default: 115200).
    - timeout: Timeout in seconds for the serial connection (default: 5).
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Connected to {port} at {baudrate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None


def send_command(ser, command: str):
    if ser is not None:
        try:
            ser.write((command + '\r\n').encode())
            time.sleep(0.1)
            # print(f"Command sent: {command}")
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
    else:
        print("Serial connection not established.")


def read_response(ser, timeout=15) -> str:
    """
    Read the response from the NanoVNA with a timeout mechanism.
    Args:
    - ser: The serial object.
    - timeout: Time in seconds to wait for the response.
    Returns the filtered response.
    """
    start_time = time.time()
    response = ""

    if ser is not None:
        while True:
            time.sleep(0.1)  # Delay to allow data to accumulate
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                response += data
                if "ch>" in data:  # NanoVNA prompt signals end of response
                    break
            if time.time() - start_time > timeout:  # Timeout condition
                print("Warning: Read timed out")
                break
    else:
        print("Serial connection not established.")

    lines = response.splitlines()
    filtered_lines = [line for line in lines if not (line.startswith(('version', 'scanraw', 'ch>')))]
    return '\n'.join(filtered_lines)


def get_version_info(ser):
    """
    Send the 'version' command to the NanoVNA to retrieve version information.
    Args:
    - ser: The serial object representing the connection to the NanoVNA.
    Returns:
    - The version information as a string.
    """
    send_command(ser, 'version')  # Send 'version' command to the NanoVNA
    return read_response(ser)  # Read and return the response


def single_sweep_for_testing():
    # Define sweep parameters
    start_freq = 6_500_000
    end_freq = 15_000_000
    points = 50  # Number of points in the sweep
    step_size = int((end_freq - start_freq) / points)
    average = 1

    # wide sweep
    nonCalibratedNetwork, execution_time = sweep_to_non_calibrated_network(ser, start_freq, step_size, points,
                                                                           average)
    # print(f"Time taken for the sweep: {execution_time:.2f} seconds")
    calibratedNework = apply_calibration_to_network(nonCalibratedNetwork)
    plot_S11(calibratedNework)

    # Analyze the network and get the first dip and automatic span
    result = analyze_network_first_dip_with_auto_span(calibratedNework, swr_threshold=5.0, swr_span_threshold=6.0)
    ws_frequency_array, ws_s11_db_array, ws_swr_array, ws_min_s11_db, ws_min_swr, ws_freq_at_min_s11, ws_freq_at_min_swr, auto_start_freq, auto_stop_freq = result

    # Narrow Sweep
    start_freq = auto_start_freq
    end_freq = auto_stop_freq
    points = 300  # Number of points in the sweep
    step_size = int((end_freq - start_freq) / points)
    average = 1

    nonCalibratedNetwork, execution_time = sweep_to_non_calibrated_network(ser, start_freq, step_size, points,
                                                                           average)
    print(f"Time taken for the Narrow sweep: {execution_time:.2f} seconds")
    calibratedNework = apply_calibration_to_network(nonCalibratedNetwork)
    # Analyze the network and get the first dip and automatic span
    result = analyze_network_first_dip_with_auto_span(calibratedNework, swr_threshold=5.0, swr_span_threshold=6.0)
    ns_frequency_array, ns_s11_db_array, ns_swr_array, ns_min_s11_db, ns_min_swr, ns_freq_at_min_s11, ns_freq_at_min_swr, auto_start_freq, auto_stop_freq = result

    plot_S11(calibratedNework)
    plot_swr(calibratedNework)


# --------------------------------------------------
# Core Functions
# --------------------------------------------------

def sweep_to_non_calibrated_network(ser, start_freq, step_size, points, average, retries=3, delay=2,
                                    network_name="NanoVNA_S11_Non_Calibrated"):
    """
    Execute the 'scanraw' command on the NanoVNA, retrieve S11 data, and return an skrf.Network object along with
    the time taken to execute the sweep, retrying if no valid data is received.

    Args:
    - ser: Serial object for communication with the NanoVNA.
    - start_freq: Start frequency in Hz.
    - step_size: Frequency step size in Hz.
    - points: Number of measurement points.
    - average: Number of averages.
    - retries: Number of retries if no data is received.
    - delay: Delay in seconds between retries.
    - network_name: Name of the network (optional, default is 'NanoVNA_S11_Non_Calibrated').

    Returns:
    - A tuple containing:
        - An skrf.Network object representing the S11 data.
        - The time taken to execute the function (in seconds).
    """

    attempt = 0
    while attempt < retries:
        # Start the timer
        start_time = time.time()

        # Send the scanraw command to the NanoVNA
        command = f"scanraw 0 {start_freq} {step_size} {points} {average}\r\n"
        send_command(ser, command)

        # Read the response from the NanoVNA
        response = read_response(ser)

        # Parse S11 data from the response (assuming tab-separated real and imaginary parts)
        data = []
        for line in response.splitlines():
            try:
                real, imag = map(float, line.split('\t'))
                data.append((real, imag))
            except ValueError:
                continue  # Skip invalid lines

        if data:
            # Convert parsed data to numpy array
            s11_data = np.array(data)

            # Create an array of frequencies based on start frequency, step size, and number of points
            frequencies = np.linspace(start_freq, start_freq + step_size * (points - 1), points)

            # Convert real and imaginary parts to complex S11 data
            s11_complex = s11_data[:, 0] + 1j * s11_data[:, 1]

            # Create the skrf Network object for the S11 data
            frequency_obj = rf.Frequency.from_f(frequencies, unit='Hz')
            s_parameters = s11_complex.reshape((-1, 1, 1))  # S11 is a one-port network

            # Create the Network object
            network = rf.Network(frequency=frequency_obj, s=s_parameters, name=network_name)

            # End the timer
            end_time = time.time()

            # Calculate the execution time
            execution_time = end_time - start_time

            # Return the Network object and the time taken to execute the function
            # print(f"Time taken for the sweep: {execution_time:.2f} seconds")
            return network, execution_time

        else:
            print(f"No valid data received. Retrying... (Attempt {attempt + 1}/{retries})")
            message = str(str(latest_zoomed_resonance_frequency) + " --- error line 1008")
            # Open the file in append mode and write the message
            with open("log.txt", 'a') as file:
                file.write(message + '\n')
            time.sleep(delay)
            attempt += 1

    # If all attempts fail, return None and 0 seconds
    print(f"Failed to retrieve valid data after {retries} attempts.")
    return None, 0


def apply_calibration_to_network(non_calibrated_network: rf.Network,
                                 network_name: str = "NanoVNA_S11_Calibrated") -> rf.Network:
    """
    Apply SOL (Short, Open, Load) calibration to a non-calibrated network object.

    Args:
    - non_calibrated_network: An skrf.Network object containing the raw S11 data (non-calibrated).
    - network_name: Optional. The name for the calibrated network (default: 'Calibrated_Network').

    Returns:
    - The calibrated skrf.Network object.
    """

    # Extract the frequency array from the non-calibrated network
    frequencies = non_calibrated_network.frequency.f  # Frequency array in Hz
    # Save frequencies to a CSV file for debugging (overwriting the file each time)
    # np.savetxt("frequencies_debug.csv", frequencies, delimiter=",")

    # Load measured calibration standards
    measured_short = rf.Network(os.path.join(calibration_kit_dir, 'SHORT_cal.s1p'))
    measured_open = rf.Network(os.path.join(calibration_kit_dir, 'OPEN_cal.s1p'))
    measured_load = rf.Network(os.path.join(calibration_kit_dir, 'LOAD_cal.s1p'))

    # Resample the calibration data to match the frequencies of the non-calibrated network
    measured_short_resampled = measured_short.interpolate(frequencies)
    measured_open_resampled = measured_open.interpolate(frequencies)
    measured_load_resampled = measured_load.interpolate(frequencies)

    # Create ideal Short, Open, and Load standards
    ideal_short = rf.Network(frequency=rf.Frequency.from_f(frequencies, unit='Hz'),
                             s=-1 * np.ones((len(frequencies), 1, 1)))
    ideal_open = rf.Network(frequency=rf.Frequency.from_f(frequencies, unit='Hz'), s=np.ones((len(frequencies), 1, 1)))
    ideal_load = rf.Network(frequency=rf.Frequency.from_f(frequencies, unit='Hz'), s=np.zeros((len(frequencies), 1, 1)))

    # Perform OnePort calibration using measured and ideal standards
    calibration = rf.OnePort(
        ideals=[ideal_short, ideal_open, ideal_load],
        measured=[measured_short_resampled, measured_open_resampled, measured_load_resampled]
    )
    calibration.run()

    # Apply the calibration to the non-calibrated network
    calibrated_dut = calibration.apply_cal(non_calibrated_network)

    # Set the name of the calibrated network
    calibrated_dut.name = network_name

    return calibrated_dut


def run_scanraw_to_network_file(ser, start_freq, step_size, points, average, touchstone_filename) -> rf.Network:
    """
    Execute the 'scanraw' command on the NanoVNA, retrieve the S11 data, and create an skrf.Network object.
    Args:
    - ser: Serial object for communication with the NanoVNA.
    - start_freq: Start frequency in Hz.
    - step_size: Frequency step size in Hz.
    - points: Number of measurement points.
    - average: Number of averages.
    - touchstone_filename: Filename to save the Touchstone (.s1p) file.
    """
    command = f"scanraw 0 {start_freq} {step_size} {points} {average}\r\n"
    send_command(ser, command)
    response = read_response(ser)

    # Parse the response and extract S11 data
    data = []
    for line in response.splitlines():
        try:
            real, imag = map(float, line.split('\t'))
            data.append((real, imag))
        except ValueError:
            continue  # Skip invalid lines

    if not data:
        print("No valid data received.")
        return None

    # Create S11 complex data and frequency array
    s11_data = np.array(data)
    frequencies = np.linspace(start_freq, start_freq + step_size * points, points)
    s11_complex = s11_data[:, 0] + 1j * s11_data[:, 1]

    # Create skrf Network object and save as a Touchstone file
    frequency_obj = rf.Frequency.from_f(frequencies, unit='Hz')
    s_parameters = s11_complex.reshape((-1, 1, 1))
    network = rf.Network(frequency=frequency_obj, s=s_parameters)

    save_network_to_file(network, touchstone_filename + ".s1p")
    return network


def apply_calibration(s11_data, start_freq, step_size, points) -> rf.Network:
    """
    Apply SOL (Short, Open, Load) calibration to raw S11 data.
    Args:
    - s11_data: Raw S11 data from the NanoVNA.
    - start_freq: Starting frequency in Hz.
    - step_size: Frequency step size in Hz.
    - points: Number of points in the sweep.
    Returns the calibrated Network object.
    """
    frequencies = np.linspace(start_freq, start_freq + step_size * points, points)

    # Load measured calibration standards
    measured_short = rf.Network('/CalibrationKit/SHORT_cal.s1p')
    measured_open = rf.Network('/CalibrationKit/OPEN_cal.s1p')
    measured_load = rf.Network('/CalibrationKit/LOAD_cal.s1p')

    # Resample the calibration data to match the current sweep frequencies
    measured_short_resampled = measured_short.interpolate(frequencies)
    measured_open_resampled = measured_open.interpolate(frequencies)
    measured_load_resampled = measured_load.interpolate(frequencies)

    # Create ideal Short, Open, and Load standards
    ideal_short = rf.Network(frequency=rf.Frequency.from_f(frequencies, unit='Hz'),
                             s=-1 * np.ones((len(frequencies), 1, 1)))
    ideal_open = rf.Network(frequency=rf.Frequency.from_f(frequencies, unit='Hz'), s=np.ones((len(frequencies), 1, 1)))
    ideal_load = rf.Network(frequency=rf.Frequency.from_f(frequencies, unit='Hz'), s=np.zeros((len(frequencies), 1, 1)))

    # Perform OnePort calibration
    calibration = rf.OnePort(ideals=[ideal_short, ideal_open, ideal_load],
                             measured=[measured_short_resampled, measured_open_resampled, measured_load_resampled])
    calibration.run()

    # Create the DUT Network object and apply calibration
    s11_complex = s11_data[:, 0] + 1j * s11_data[:, 1]
    dut = rf.Network(frequency=rf.Frequency.from_f(frequencies, unit='Hz'), s=s11_complex.reshape((-1, 1, 1)))
    calibrated_dut = calibration.apply_cal(dut)

    return calibrated_dut


def analyze_network_first_dip_with_auto_span(network: rf.Network, swr_threshold: float = 5.0,
                                             swr_span_threshold: float = 4.0):
    """
    Simplified function to analyze a skrf.Network object, find the first S11/SWR dip, and determine an automatic span.

    Args:
    - network: skrf.Network object to analyze.
    - swr_threshold: Threshold for SWR values to find the dip (default: 5.0).
    - swr_span_threshold: SWR threshold for determining the span around the first dip (default: 4.0).

    Returns:
    - frequency_array: Array of frequencies in Hz.
    - s11_db_array: Array of S11 magnitudes in dB.
    - swr_array: Array of SWR values.
    - min_s11_db: Minimum S11 value in dB.
    - min_swr: Minimum SWR value.
    - freq_at_min_s11: Frequency corresponding to minimum S11 value.
    - freq_at_min_swr: Frequency corresponding to minimum SWR value.
    - auto_start_freq: Start frequency for the span where SWR < swr_span_threshold.
    - auto_stop_freq: Stop frequency for the span where SWR < swr_span_threshold.
    """

    # Extract frequency array and S11 in dB
    frequency_array = network.frequency.f
    s11_db_array = 20 * np.log10(np.abs(network.s[:, 0, 0]))
    # Compute SWR using skrf's s_vswr property
    swr_array = network.s_vswr[:, 0, 0]
    # Find the first dip (local minima) in S11 and SWR where SWR < swr_threshold
    min_idx = np.argmin(s11_db_array)
    min_s11_db = s11_db_array[min_idx]
    freq_at_min_s11 = frequency_array[min_idx]
    min_swr = swr_array[min_idx]
    freq_at_min_swr = frequency_array[min_idx]

    # Calculate the span where SWR < swr_span_threshold
    swr_below_threshold = swr_array < swr_span_threshold
    if np.any(swr_below_threshold):
        auto_start_freq = frequency_array[np.where(swr_below_threshold)[0][0]]
        auto_stop_freq = frequency_array[np.where(swr_below_threshold)[0][-1]]
    else:
        auto_start_freq = frequency_array[0]
        auto_stop_freq = frequency_array[-1]

    # Center the span around the minimum S11 frequency
    half_span = max(freq_at_min_s11 - auto_start_freq, auto_stop_freq - freq_at_min_s11)
    auto_start_freq = freq_at_min_s11 - half_span
    auto_stop_freq = freq_at_min_s11 + half_span

    return (
    frequency_array, s11_db_array, swr_array, min_s11_db, min_swr, freq_at_min_s11, freq_at_min_swr, auto_start_freq,
    auto_stop_freq)


def save_network_to_file(network: rf.Network, filename: str, folder_name: str = "CalibrationKit"):
    """
    Save an skrf.Network object to a specified folder.
    If the folder does not exist, it is created.
    Args:
    - network: The skrf.Network object to save.
    - filename: The name of the file to save (e.g., 'calibration.s1p').
    - folder_name: The subfolder where the file should be saved (default: 'CalibrationKit').
    """
    folder_path = os.path.join(os.getcwd(), folder_name)
    os.makedirs(folder_path, exist_ok=True)

    file_path = os.path.join(folder_path, filename)
    network.write_touchstone(file_path)
    print(f"Network saved to {file_path}")


def generate_calibration_kit():
    # Define sweep parameters for the calibration network
    start_freq = 3_000_000
    end_freq = 30_000_000
    points = 2000  # Number of points in the sweep
    step_size = int((end_freq - start_freq) / points)
    average = 1  # Average 1 time
    # Prompt for Open Calibration
    input("Please connect the Open calibration standard to the NanoVNA port CH0, then press Enter to continue...")
    touchstone_filename = "OPEN_cal"
    skrf_network = run_scanraw_to_network_file(ser, start_freq, step_size, points, average, touchstone_filename)
    # Prompt for Short Calibration
    input("Please connect the Short calibration standard to the NanoVNA port CH0, then press Enter to continue...")
    touchstone_filename = "SHORT_cal"
    skrf_network = run_scanraw_to_network_file(ser, start_freq, step_size, points, average, touchstone_filename)
    # Prompt for Load Calibration
    input("Please connect the Load calibration standard to the NanoVNA port CH0, then press Enter to continue...")
    touchstone_filename = "LOAD_cal"
    skrf_network = run_scanraw_to_network_file(ser, start_freq, step_size, points, average, touchstone_filename)
    sys.exit()


def continuous_sweeping_thread():
    global ws_frequency_array, ws_s11_db_array, ws_swr_array, ws_min_s11_db, ws_min_swr, ws_freq_at_min_s11, ws_freq_at_min_swr
    global ns_frequency_array, ns_s11_db_array, ns_swr_array, ns_min_s11_db, ns_min_swr, ns_freq_at_min_s11, ns_freq_at_min_swr
    global sweepNumber
    global latest_zoomed_resonance_frequency
    sweepNumber = 0

    while True:
        # wide sweep
        start_freq = 6_500_000
        end_freq = 15_000_000
        points = 600  # Number of points in the sweep
        step_size = int((end_freq - start_freq) / points)
        average = 1
        print("\nPerforming Wide Sweep (Sweep n°" + str(sweepNumber) + ")")
        nonCalibratedNetwork, execution_time = sweep_to_non_calibrated_network(ser, start_freq, step_size, points,
                                                                               average)
        try:
            calibratedNework = apply_calibration_to_network(nonCalibratedNetwork)
            # Analyze the network and get the first dip and automatic span for next "Narrow" Sweep
            result = analyze_network_first_dip_with_auto_span(calibratedNework, swr_threshold=5.0,
                                                              swr_span_threshold=6.0)
            ws_frequency_array, ws_s11_db_array, ws_swr_array, ws_min_s11_db, ws_min_swr, ws_freq_at_min_s11, ws_freq_at_min_swr, auto_start_freq, auto_stop_freq = result
            # Narrow Sweep
            start_freq = auto_start_freq
            end_freq = auto_stop_freq
            points = 400  # Number of points in the sweep
            step_size = int((end_freq - start_freq) / points)
            average = 1
            print("\nPerforming Narrow Sweep (Sweep n°" + str(sweepNumber) + ")")
            nonCalibratedNetwork, execution_time = sweep_to_non_calibrated_network(ser, start_freq, step_size, points,
                                                                                   average)
            calibratedNework = apply_calibration_to_network(nonCalibratedNetwork)
            # Analyze the network
            result = analyze_network_first_dip_with_auto_span(calibratedNework, swr_threshold=5.0,
                                                              swr_span_threshold=6.0)
            ns_frequency_array, ns_s11_db_array, ns_swr_array, ns_min_s11_db, ns_min_swr, ns_freq_at_min_s11, ns_freq_at_min_swr, auto_start_freq, auto_stop_freq = result

            # Apply filtering
            ns_swr_array = savgol_filter(ns_swr_array, window_length=21, polyorder=3)
            # Find the index of the minimum value in the filtered S11 data
            min_index = np.argmin(ns_swr_array)
            # Get the corresponding frequency for this minimum value
            ns_freq_at_min_swr = ns_frequency_array[min_index]

            ns_s11_db_array = savgol_filter(ns_s11_db_array, window_length=21, polyorder=3)
            min_index = np.argmin(ns_s11_db_array)
            ns_freq_at_min_s11 = ns_frequency_array[min_index]

            latest_zoomed_resonance_frequency = int(ns_freq_at_min_s11)
            sweepNumber = sweepNumber + 1
            print("Sweep:", sweepNumber, "completed")
        except:
            message = "Error at Sweep number:" + str(sweepNumber)

            print(message)  # Open the file in append mode and write the message
            # with open("log.txt", 'a') as file:
            # file.write(message + '\n')
            pass


# --------------------------------------------------
# Plotting Functions For De-Bugging
# --------------------------------------------------
def plot_S11(network: rf.Network):
    """
    Plot the S11 parameter (in dB) of a given skrf.Network object and include detailed network info in the title.

    Args:
    - network: skrf.Network object to plot.
    """
    # Extract frequency array (in Hz) and S11 (complex values)
    frequencies = network.frequency.f
    s11_db = 20 * np.log10(np.abs(network.s[:, 0, 0]))  # S11 in dB

    # Extract network information
    start_freq = frequencies[0] / 1e6  # Convert to MHz
    end_freq = frequencies[-1] / 1e6  # Convert to MHz
    points = len(frequencies)
    z0 = network.z0[0]  # Characteristic impedance (assuming it's the same for all points)

    # Construct the plot title
    title = f"1-Port Network: '{network.name}', {start_freq:.1f}-{end_freq:.1f} MHz, {points} pts, z0={z0}"

    # Create a Matplotlib plot
    plt.figure(figsize=(10, 6))
    plt.plot(frequencies / 1e6, s11_db, label=network.name, color='blue')  # Convert Hz to MHz for x-axis

    # Add labels, title, and grid
    plt.title(title)
    plt.xlabel("Frequency (MHz)")
    plt.ylabel("S11 (dB)")
    plt.grid(True)
    plt.legend()

    # Show the plot
    plt.show()


def plot_swr(network: rf.Network):
    """
    Plot the SWR (Standing Wave Ratio) for a given skrf.Network object using the built-in s_vswr property.

    Args:
    - network: skrf.Network object to plot.
    """

    # Get frequency array in Hz and convert to MHz for plotting
    frequencies = network.frequency.f / 1e6  # Convert Hz to MHz

    # Use the built-in s_vswr property to get the SWR array
    swr_array = network.s_vswr[:, 0, 0]

    # Create the plot
    plt.figure(figsize=(10, 6))
    plt.plot(frequencies, swr_array, label="SWR", color='purple')

    # Add plot labels and title
    plt.title(f"SWR Plot for {network.name}")
    plt.xlabel("Frequency (MHz)")
    plt.ylabel("SWR")
    plt.grid(True)
    plt.legend()

    # Display the plot
    plt.show()


# Function to log data every minute
def record_resonance_frequency():
    # Open the file in append mode, so we don't overwrite existing data
    with open(longtime_recording, 'a', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        # Write headers if the file is new
        if csvfile.tell() == 0:
            csvwriter.writerow(['Time', 'Zoomed_Resonance_Frequency'])

        while True:
            # Get the current time in a format suitable for Highcharts (e.g., UNIX timestamp)
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            # Write the current time and frequency value to the CSV file
            csvwriter.writerow([current_time, latest_zoomed_resonance_frequency])
            csvfile.flush()  # Ensure data is written to disk
            time.sleep(60)  # Wait for 1 minute


# --------------------------------------------------
# Main Program Flow
# --------------------------------------------------
ser = None  # Initialize ser as None to avoid issues in the exception handler

if __name__ == '__main__':
    print("Hello")
    # uploadNewLookUpTableToESP32()
    # sys.exit()
    try:
        serial_port = get_vna_port()  # Auto-detect NanoVNA port
        ser = initialize_serial(serial_port)  # Initialize the connection
        version_info = get_version_info(ser)  # Retrieve NanoVNA version
        print("NanoVNA Version Info:", version_info)
        # Uncomment to regenerate a calibration kit
        # generate_calibration_kit()
        # single_sweep_for_testing()
    except OSError as e:
        print(f"Error: {e}")  # Display error when NanoVNA is not found
        print("NanoVNA device not found. Exiting application.")
        sys.exit(1)  # Exit the application if NanoVNA is not found
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        if ser:  # Close the serial connection only if it was initialized
            ser.close()
            print("Serial connection closed.")
        sys.exit(1)  # Exit with an error code for unexpected exceptions

    # Start the sweep thread if NanoVNA was successfully initialized
    sweep_thread = threading.Thread(target=continuous_sweeping_thread)
    sweep_thread.daemon = True
    sweep_thread.start()

    while sweepNumber == 0:
        print("Wait for 1st sweep to complete")
        time.sleep(1)

    # Suppress Flask default logging
    Flasklog = logging.getLogger('werkzeug')
    Flasklog.setLevel(logging.ERROR)  # Set to ERROR to suppress INFO level logs
    # Start the logging thread
    # This is to observe analyse resonance frequency fluctuations during days without moving capacitor
    logging_thread = threading.Thread(target=record_resonance_frequency, daemon=True)


    # logging_thread.start()

    # Function to run Flask app
    def run_flask():
        app.run(host='127.0.0.1', use_reloader=False, port=5558)


    # Start Flask server in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.start()

    # Automatically open the web page in the default browser
    webbrowser.open("http://127.0.0.1:5558")

    # Wait for the Flask server thread to complete
    flask_thread.join()

    # Cleanup serial connection
    if ser:
        ser.close()
        print("Serial connection closed.")
