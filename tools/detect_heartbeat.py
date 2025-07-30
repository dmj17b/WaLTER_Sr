import can

bus0 = can.interface.Bus(channel='can0', interface='socketcan')
bus1 = can.interface.Bus(channel='can1', interface='socketcan')

print("Listening for ODrive heartbeat frames (0x001, 0x041, etc)")
try:
    while True:
        msg0 = bus0.recv()
        msg1 = bus1.recv()
        if msg0.arbitration_id & 0x1F == 0x01:
            node_id = msg0.arbitration_id >> 5
            print(f"Heartbeat from node {node_id}:", msg0.data.hex())
        if msg1.arbitration_id & 0x1F == 0x01:
            node_id = msg1.arbitration_id >> 5
            print(f"Heartbeat from node {node_id}:", msg1.data.hex())
except KeyboardInterrupt:
    print("Stopped")
