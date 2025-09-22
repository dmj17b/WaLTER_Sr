### Tools

This folder holds miscellaneous python/shell scripts and data files that can come in handy when working with WaLTER Sr.

## m6c12_calibration.py

This python script is to be used to calibrate ODrives for MAD M6C12 motor using built-in encoder. The script will set general parameters and then run the calibration sequence. It is important to make sure any (asymmetric) loads are removed from the motor before calibration. The NODEID is the only variable that should need to change between different motor calibrations. To calibrate, ensure motor is plugged into a power supply and the host computer through USB-C port. It is important to note that this script only works through USB, not CAN.

# m6c12.config
Text file to hold config data for MAD M6C12 motors. This file is not referenced by any other programs and is only included for user reference. It will also not update with modifications to calibration or param mod script.

## ParamModCAN.py

Python script used to modify ODrive parameters over CAN lines. References flat_endpoints.json to know what messages to send. Use this to update gains, limits, etc. just make sure to not overwrite any important calibration data. This is intended to be used on the fully assembled robot whenever needed, as opposed to the calibration script which is only run when motors are separated from the gear train and connected to the host computer via USB-C port.

## rezero_can.py

Makes use of the same communication technique/class as ParamModCAN.py only this script is simplified to only re-zero the joints by reading the current motor position and changing the offset for the pos_vel_mapper to this position. Before running, make sure legs are in the desired initial position (T-pose).

## load_can.sh

This script is used to set up the two CAN busses on the Orin. If there are any random errors in CAN communication, you can run this script to reset the CAN sockets