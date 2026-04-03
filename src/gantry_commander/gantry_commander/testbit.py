import serial
import time

def _build_packet(cmd: int, data: bytes) -> bytes:
    body = bytes([0x7A, cmd]) + data
    chk = 0
    for b in body:
        chk ^= b
    return body + bytes([chk, 0x7B])

# Potential Status Commands for this 0x7A protocol
# 0x01 is often "Read Status/Position"
# 0x09 is often "Read Error/Stall"
CMD_QUERY_V1 = _build_packet(0x01, bytes([0x00]*8))
CMD_QUERY_V2 = _build_packet(0x09, bytes([0x00]*8))

def test_query(ser, label, packet):
    print(f"\nTesting {label}...")
    ser.reset_input_buffer()
    ser.write(packet)
    time.sleep(0.1)
    if ser.in_waiting > 0:
        res = ser.read(ser.in_waiting)
        print(f"RESPONSE (HEX): {' '.join([f'{b:02X}' for b in res])}")
        return True
    print("No response.")
    return False

def main():
    try:
        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)
        print("Connected to /dev/ttyACM2")
        
        test_query(ser, "Command 0x01", CMD_QUERY_V1)
        test_query(ser, "Command 0x09", CMD_QUERY_V2)
        
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()