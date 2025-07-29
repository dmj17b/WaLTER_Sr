
# -- start load
import json
import time
import struct
import can
import atexit




# Add the parent directory to the path so we can import flat_endpoints.json
with open('flat_endpoints.json', 'r') as f:
    endpoint_data = json.load(f)
    endpoints = endpoint_data['endpoints']
# -- end load

# -- start definitions
OPCODE_READ = 0x00
OPCODE_WRITE = 0x01

# See https://docs.python.org/3/library/struct.html#format-characters
format_lookup = {
    'bool': '?',
    'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h',
    'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q',
    'float': 'f'
}


class ODriveModifier:
    def __init__(self, node_id=0, channel = "can1"):
        self.node_id = node_id
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")
        self.bus.__enter__()
        atexit.register(lambda: self.bus.__exit__(None, None, None))
        self.endpoints = endpoints
        self.format_lookup = format_lookup
        self.path_vi_limit = 'axis0.controller.config.vel_integrator_limit'
        self.path_vel_gain = 'axis0.controller.config.vel_gain'
        self.path_vi_gain = 'axis0.controller.config.vel_integrator_gain'
        self.path_pos_gain = 'axis0.controller.config.pos_gain'
        self.trap_traj_vel_limit = 'axis0.trap_traj.config.vel_limit'


    # Function to check ODrive version against endpoint data
    def check_version(self):
        """Check the ODrive version and return it."""
        # Flush CAN RX buffer so there are no more old pending messages
        while not (self.bus.recv(timeout=0) is None): pass
        # Send read command
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x00), # 0x00: Get_Version
            data=b'',
            is_extended_id=False
        ))
        # Await reply
        for msg in self.bus:
            if msg.is_rx and msg.arbitration_id == (self.node_id << 5 | 0x00):
                break
        # Unpack and return reply
        _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)
        # If one of these asserts fail, you're probably not using the right flat_endpoints.json file
        assert endpoint_data['fw_version'] == f"{fw_major}.{fw_minor}.{fw_revision}"
        assert endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"

    def write(self, path, value):
        """Write a value to the ODrive."""
        endpoint_id = self.endpoints[path]['id']
        endpoint_type = self.endpoints[path]['type']

        # Flush CAN RX buffer so there are no more old pending messages
        while not (self.bus.recv(timeout=0) is None): pass

        # Send write command
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x04), # 0x04: RxSdo
            data=struct.pack('<BHB' + self.format_lookup[endpoint_type], OPCODE_WRITE, endpoint_id, 0, value),
            is_extended_id=False
        ))

        # Await confirmation (if firmware >= 0.6.11)
        for msg in self.bus:
            if msg.is_rx and msg.arbitration_id == (self.node_id << 5 | 0x05):
                break
        time.sleep(0.01)  # Wait a bit to ensure the write is processed
    

    def read(self, path):
        """Read a value from the ODrive."""
        endpoint_id = self.endpoints[path]['id']
        endpoint_type = self.endpoints[path]['type']

        # Flush CAN RX buffer so there are no more old pending messages
        while not (self.bus.recv(timeout=0) is None): pass

        # Send read command
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x04), # 0x04: RxSdo
            data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
            is_extended_id=False
        ))

        # Await reply
        for msg in self.bus:
            if msg.is_rx and msg.arbitration_id == (self.node_id << 5 | 0x05):
                break
        # Unpack and return reply
        _, _, _, return_value = struct.unpack_from('<BHB' + self.format_lookup[endpoint_type], msg.data)
        time.sleep(0.01)
        return return_value
    
    
    def save_config(self):
        """Save the current configuration to flash."""
        path = "save_configuration"
        endpoint_id = self.endpoints[path]['id']

        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x04), # 0x04: RxSdo
            data=struct.pack('<BHB', OPCODE_WRITE, endpoint_id, 0),
            is_extended_id=False
        ))



fr_knee = ODriveModifier(node_id=0, channel="can1")
fl_knee = ODriveModifier(node_id=2, channel="can1")
rl_knee = ODriveModifier(node_id=4, channel="can0")
rr_knee = ODriveModifier(node_id=6, channel="can0")

fr_hip = ODriveModifier(node_id=1, channel="can1")
fl_hip = ODriveModifier(node_id=3, channel="can1")
rl_hip = ODriveModifier(node_id=5, channel="can0")
rr_hip = ODriveModifier(node_id=7, channel="can0")


''' Update Knee Parameters'''
for motor in [fr_knee, fl_knee, rl_knee, fr_hip, fl_hip, rl_hip, rr_knee, rr_hip]:
    motor.check_version()  # Check ODrive version

    ''' Check CAN communication rates'''
    print(f"Node ID: {motor.node_id}")
    print(f"Version MSG Rate: {motor.read('axis0.config.can.version_msg_rate_ms')}")
   
    motor.pos = motor.read('axis0.pos_vel_mapper.pos_rel')
    print(f"Motor Position: {motor.pos}")
    print("\n")
    motor.write('axis0.pos_vel_mapper.config.offset', motor.pos)  # Set position relative to 0

    # Save configuration to flash memory
    motor.save_config()
    print(f"Updated communication rates and saved configuration for Node ID: {motor.node_id}")

