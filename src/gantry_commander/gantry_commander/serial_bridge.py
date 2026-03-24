import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import threading
import struct 

# ========================
# GRIPPER PROTOCOL CLASS
# ========================
def _build_packet(cmd: int, data: bytes) -> bytes:
    assert len(data) == 8, "data must be 8 bytes"
    body = bytes([0x7A, cmd]) + data
    chk = 0
    for b in body:
        chk ^= b
    return body + bytes([chk, 0x7B])

class Gripper:
    CMD_ENABLE = _build_packet(0x08, bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    
    CMD_OPEN = _build_packet(0x0C, bytes([0x01, 0x13, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00]))
    CMD_CLOSE = _build_packet(0x0C, bytes([0x02, 0x13, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00]))
    
    CMD_STOP = _build_packet(0x0C, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.1)
        self._send(self.CMD_ENABLE)

    def _send(self, packet: bytes):
        self.ser.write(packet)

    def set_speed(self, speed: int):
        speed = max(100, min(9999, speed))
        data = struct.pack(">H", speed) + bytes(6)
        self._send(_build_packet(0x0D, data))
        self._send(_build_packet(0x0E, data))

    def open(self, duration=1.5):
        self._send(self.CMD_OPEN)
        print(f"[Gripper] Opening for {duration} seconds...")
        threading.Timer(duration, self.stop).start()

    def close(self, duration=1.5):
        self._send(self.CMD_CLOSE)
        print(f"[Gripper] Closing for {duration} seconds...")
        threading.Timer(duration, self.stop).start()

    def stop(self):
        self._send(self.CMD_STOP)
        print("[Gripper] Motor Halted.")

    def disconnect(self):
        self.ser.close()

# =======================
# YOUR MAIN BRIDGE NODE
# =======================
class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.is_ready = False
        self.streaming_unlocked = False
        
        self.gripper_state = 0
        self.gripper = None
        try:
            self.gripper = Gripper('/dev/ttyACM0', 115200)
            self.gripper.set_speed(800)
            self.get_logger().info("Gripper connected.")
        except Exception as e:
            self.get_logger().error(f"Gripper Error: {e}")

        try:
            self.ser = serial.Serial('/dev/arduino_gantry', 115200, timeout=0.1)
            self.get_logger().info("Serial connection established.")
            time.sleep(2.0)
            threading.Thread(target=self.read_serial, daemon=True).start()
            threading.Thread(target=self.user_interface, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")

        self.subscription = self.create_subscription(
            JointState, 
            'joint_states', 
            self.listener_callback, 
            10)

    def read_serial(self):
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"\n[ARDUINO]: {line}")
                    if "HOMING_COMPLETE" in line:
                        self.is_ready = True

    def user_interface(self):
        while rclpy.ok():
            print("\n--- MOVEIT GANTRY BRIDGE ---")
            print("H: Home Gantry (Required)")
            print("T: 1000 Step Test")
            print("C: Toggle Gripper")
            print("S: EMERGENCY STOP")
            print("R: ENABLE MOVEIT STREAMING")
            choice = input("Select Action: ").upper()

            if choice == 'H':
                self.streaming_unlocked = False
                self.ser.write(b'H\n')
            elif choice == 'T':
                self.ser.write(b'T\n')
            
            elif choice == 'C':
                if self.gripper:
                    if self.gripper_state == 0:
                        self.gripper.open()
                        self.gripper_state = 1
                        print("[Gripper] State: 1 (Opened)")
                    else:
                        self.gripper.close()
                        self.gripper_state = 0
                        print("[Gripper] State: 0 (Closed)")
                else:
                    print("Error: Gripper not connected!")
                    
            elif choice == 'S':
                self.ser.write(b'S\n')
                if self.gripper: self.gripper.stop()
                self.is_ready = False
                self.streaming_unlocked = False
            elif choice == 'R':
                if self.is_ready:
                    print(">>> NOW FOLLOWING MOVEIT TRAJECTORIES <<<")
                    self.streaming_unlocked = True
                else:
                    print("Error: You must Home 'H' first!")

    def listener_callback(self, msg):
        if self.is_ready and self.streaming_unlocked:
            try:
                idx_x = msg.name.index('x_axis_joint')
                idx_y = msg.name.index('y_axis_joint')
                idx_z = msg.name.index('z_axis_lift_joint')

                x = msg.position[idx_x]
                y = msg.position[idx_y]
                z = msg.position[idx_z]
                
                command = f"X{x:.4f}Y{y:.4f}Z{z:.4f}\n"
                self.ser.write(command.encode('utf-8'))
                
                print(f"Sent to Arduino: {command.strip()}", end='\r')

            except (ValueError, IndexError) as e:
                print(f"Waiting for joints... Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    
    if node.gripper:
        node.gripper.disconnect()
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()