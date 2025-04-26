import socket
import struct
import threading
import time
from queue import Queue, Empty

commands = {
    "START": {"id": 0, "length": 0, "dtype": None},
    "KILL": {"id": 1, "length": 0, "dtype": None},
    "HOVER": {"id": 2, "length": 0, "dtype": None},
    "SET_KP_ROLL": {"id": 3, "length": 4, "dtype": "fp32"},
    "SET_KP_PITCH": {"id": 4, "length": 4, "dtype": "fp32"},
}

def send_udp_command(sock, addr, cmd_id, payload=None, dtype=None):
    msg = bytes([cmd_id])
    if payload and dtype:
        for val in payload:
            if dtype == "fp32":
                int_bits = struct.unpack("I", struct.pack("f", val))[0]
                msg += int_bits.to_bytes(4, byteorder="little")
            elif dtype == "int32":
                msg += int(val).to_bytes(4, byteorder="little", signed=True)
    sock.sendto(msg, addr)
    print(f"[INFO] Sent command ID {cmd_id} with payload {payload if payload else []}")

def communication_thread(drone_ip, drone_port, command_queue):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (drone_ip, drone_port)

    while True:
        try:
            cmd = command_queue.get(timeout=0.1)
            cmd_name, payload, dtype = cmd
            cmd_info = commands[cmd_name]
            send_udp_command(sock, addr, cmd_info["id"], payload, dtype)
        except Empty:
            pass
        time.sleep(0.1)

def main(drone_ip, drone_port):
    command_queue = Queue()
    thread = threading.Thread(target=communication_thread, args=(drone_ip, drone_port, command_queue), daemon=True)
    thread.start()

    while True:
        print("\nAvailable commands:", ", ".join(commands.keys()))
        cmd = input("Enter command: ").strip().upper()

        if cmd not in commands:
            print("Invalid command.")
            continue

        cmd_info = commands[cmd]
        if cmd_info["length"] == 0:
            command_queue.put((cmd, [], None))
        else:
            payload = []
            for i in range(cmd_info["length"] // 4):
                val = input(f"Enter value {i+1} for {cmd} ({cmd_info['dtype']}): ")
                payload.append(float(val) if cmd_info["dtype"] == "fp32" else int(val))
            command_queue.put((cmd, payload, cmd_info["dtype"]))

if __name__ == "__main__":
    DRONE_IP = "10.196.17.104"  # Update to match ESP32 IP
    DRONE_PORT = 80             # Same as WIFI_PORT in firmware
    main(DRONE_IP, DRONE_PORT)
