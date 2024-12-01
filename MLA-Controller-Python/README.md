
# MLA-Controller-PYTHON



make sur eto use DHCP
Install firmware for vna

## Initial Configuration

Check out the video for the initial configuration process of the MLA Controller:

[![Initial Configuration](https://img.youtube.com/vi/leM8o_F7qt0/0.jpg)](https://www.youtube.com/watch?v=leM8o_F7qt0)






# MLA Configurator - Magnetic Loop Antenna Controller

## Overview
The MLA Configurator is a Python-based application designed to automatically generate the lookup table that will be flashed to the Slave ESP32 for tuning a magnetic loop antenna. It leverages a NanoVNA (Vector Network Analyzer) and an ESP32 device to collect measurement data across a range of frequencies. The generated lookup table can be used by the control unit for quick tuning, ensuring optimal performance of the antenna.

The app uses several powerful libraries like `Flask` for server functionality, `numpy` for numerical computations, `matplotlib` for data visualization, and `scikit-learn` for data analysis and modeling. It integrates seamlessly with the NanoVNA-Q firmware and can generate calibration files and characterization data.

## Features
- **Measurement Collection**: Interfaces with the NanoVNA and ESP32 to collect S11, SWR, and other relevant data.
- **Data Processing**: Uses advanced algorithms like cubic spline interpolation and polynomial regression to create a smooth and accurate lookup table.
- **Web Interface**: Includes a Flask server for easy configuration and interaction with the hardware.
- **Characterization and Calibration**: Supports storing and managing calibration and characterization data for future use.
- **Plotting and Visualization**: Generates plots of the antenna's performance across different frequencies, helping users visualize the tuning process.

## Development Environment
The app is developed and tested exclusively on macOS. However, it has been briefly tested on Windows, and it worked as expected. It should function similarly on both platforms, but the macOS environment is the primary development setup.

## VNA (Vector Network Analyzer)
This application works with affordable NanoVNA devices, which can be found on platforms like AliExpress under the name "NanoVNA Vector Network Analyzer." 

**For optimal compatibility, the NanoVNA **must** be flashed with the NanoVNA-Q firmware**. No worries—there is no risk of damaging the device during the flashing process. You can find the firmware at the [NanoVNA-Q GitHub repository](https://github.com/qrp73/NanoVNA-Q), and a copy is included in this repository.

## Calibration
For first-time use, it is necessary to generate SOL calibration networks. You can do this by uncommenting the relevant lines in the code. Look for the section marked `# Uncomment to regenerate a calibration kit` and follow the instructions in the code.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/MLA-Configurator.git
   ```
2. Install required Python libraries:
   ```bash
   pip install -r requirements.txt
   ```

## Usage
To begin the characterization process, it is recommended to set up a virtual environment for managing dependencies. Here's how you can do that:

1. **Create a virtual environment**:
   ```bash
   python3 -m venv venv
   ```

2. **Activate the virtual environment**:
   - On macOS/Linux:
     ```bash
     source venv/bin/activate
     ```
   - On Windows:
     ```bash
     .\venv\Scripts\activate
     ```

3. **Install the required dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Run the Python script** to start the characterization process:
   ```bash
   python3 MLA_Configurator.py
   ```

For an optimal development experience, it is recommended to use an IDE like **PyCharm**. PyCharm provides great support for Python projects and virtual environments, making it easier to manage dependencies and work with the code.

## Dependencies
- `Flask`
- `numpy`
- `matplotlib`
- `scikit-learn`
- `skrf` (for handling network analyzer data)
- Other libraries as specified in the `requirements.txt` file.

## Data Storage
The application saves data to CSV files in the `CharacterizationData` directory. You can access and review these files for more detailed information on the measurements.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
