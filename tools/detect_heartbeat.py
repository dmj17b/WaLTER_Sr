import can

bus = can.interface.Bus(channel='can0', interface='socketcan')

print("Listening for ODrive heartbeat frames (0x001, 0x041, etc)")
try:
    while True:
        msg = bus.recv()
        if msg.arbitration_id & 0x1F == 0x01:
            node_id = msg.arbitration_id >> 5
            print(f"Heartbeat from node {node_id}:", msg.data.hex())
except KeyboardInterrupt:
    print("Stopped")
