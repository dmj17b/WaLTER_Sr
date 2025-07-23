
import odrive
from odrive.enums import *
import time
import math
from odrive.utils import dump_errors, request_state


NODEID = 2

print("Starting ODrive configuration.")

print("Finding ODrive connection...")
odrv0 = odrive.find_any()
print("Found ODrive with Serial #: " + f"{odrv0.serial_number:x}")

print("Erasing current configuration...")
try:
	odrv0.erase_configuration()
except Exception:
	time.sleep(3.0)
print("Reconnecting to ODrive...")
odrv0 = odrive.find_any()

print("Writing other configuration...")

odrv = odrv0
odrv.config.dc_bus_overvoltage_trip_level = 52
odrv.config.dc_bus_undervoltage_trip_level = 20
odrv.config.dc_max_positive_current = math.inf
odrv.config.dc_max_negative_current = -math.inf
odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrv.axis0.config.motor.pole_pairs = 14
odrv.axis0.config.motor.torque_constant = 0.05513333333333333
odrv.axis0.config.motor.current_soft_max = 36
odrv.axis0.config.motor.current_hard_max = 56
odrv.axis0.config.motor.calibration_current = 1
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
odrv.axis0.config.can.heartbeat_msg_rate_ms = 10
odrv.axis0.config.can.encoder_msg_rate_ms = 2
odrv.axis0.config.can.iq_msg_rate_ms = 2
odrv.axis0.config.can.torques_msg_rate_ms = 2
odrv.axis0.config.can.error_msg_rate_ms = 2
odrv.axis0.config.can.temperature_msg_rate_ms = 2
odrv.axis0.config.can.bus_voltage_msg_rate_ms = 2
odrv.axis0.config.enable_watchdog = False
odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
odrv.config.enable_uart_a = False

try:
	odrv.save_configuration()
except Exception:
	time.sleep(3.0)

# Reconnect to ODrive after saving configuration
print("Reconnecting to ODrive...")
odrv0 = odrive.find_any()

try:
    request_state(odrv0.axis0, AxisState.FULL_CALIBRATION_SEQUENCE)
    timeout = 60  # Longer timeout for full sequence
    start_time = time.time()
    
    while odrv0.axis0.current_state != AxisState.IDLE:
        if time.time() - start_time > timeout:
            print("Calibration timeout!")
            break
        try:
            state = odrv0.axis0.current_state
            print(f"Calibration state: {state}")
            time.sleep(2.0)  # Longer sleep
        except Exception as e:
            print(f"Connection lost during calibration: {e}")
            time.sleep(3.0)
            try:
                odrv0 = odrive.find_any()
                print("Reconnected to ODrive")
            except:
                print("Failed to reconnect")
                break
                
except Exception as e:
    print(f"Calibration failed: {e}")

dump_errors(odrv0)



print("Saving configuration...")
try:
	odrv0.save_configuration()
except Exception:
	time.sleep(3.0)
print("Reconnecting to ODrive...")
odrv0 = odrive.find_any()
