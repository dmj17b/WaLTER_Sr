#!/usr/bin/env python3

import odrive
from odrive.enums import *
import time
import math

NODEID = 2

print("Starting ODrive configuration.")

print("Finding ODrive connection...")
odrv0 = odrive.find_any()
print("Found ODrive with Serial #: " + f"{odrv0.serial_number:x}")

print("Erasing current configuration...")

odrv0.erase_configuration()
time.sleep(3)

print("Reconnecting to ODrive...")
odrv0 = odrive.find_any()

print("Writing other configuration...")

import time
import math
NODEID = 2

odrv = odrv0
odrv.config.dc_bus_overvoltage_trip_level = 52
odrv.config.dc_bus_undervoltage_trip_level = 10.5
odrv.config.dc_max_positive_current = math.inf
odrv.config.dc_max_negative_current = -math.inf
odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrv.axis0.config.motor.pole_pairs = 7
odrv.axis0.config.motor.torque_constant = 0.02506060606060606
odrv.axis0.config.motor.current_soft_max = 14
odrv.axis0.config.motor.current_hard_max = 28.2
odrv.axis0.config.motor.calibration_current = 10
odrv.axis0.config.motor.resistance_calib_max_voltage = 2
odrv.axis0.config.calibration_lockin.current = 10
odrv.axis0.motor.motor_thermistor.config.enabled = True
odrv.axis0.motor.motor_thermistor.config.r_ref = 5000
odrv.axis0.motor.motor_thermistor.config.beta = 3977
odrv.axis0.motor.motor_thermistor.config.temp_limit_lower = 50
odrv.axis0.motor.motor_thermistor.config.temp_limit_upper = 70
odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrv.axis0.controller.config.vel_limit = 10
odrv.axis0.controller.config.vel_limit_tolerance = 1.2
odrv.axis0.config.torque_soft_min = -math.inf
odrv.axis0.config.torque_soft_max = math.inf
odrv.axis0.trap_traj.config.accel_limit = 10
odrv.axis0.controller.config.vel_ramp_rate = 10
odrv.can.config.protocol = Protocol.SIMPLE
odrv.can.config.baud_rate = 1000000
odrv.axis0.config.can.node_id = NODEID
odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
odrv.axis0.config.can.encoder_msg_rate_ms = 10
odrv.axis0.config.can.iq_msg_rate_ms = 10
odrv.axis0.config.can.torques_msg_rate_ms = 10
odrv.axis0.config.can.error_msg_rate_ms = 10
odrv.axis0.config.can.temperature_msg_rate_ms = 10
odrv.axis0.config.can.bus_voltage_msg_rate_ms = 10
odrv.axis0.config.enable_watchdog = False
odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
odrv.config.enable_uart_a = False


print("Running motor calibration...")
odrv.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
time.sleep(0.5)
while odrv0.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

print("Saving configuration...")
odrv.save_configuration()
print("Reconnecting to ODrive...")
odrv = odrive.find_any()