import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import threading
import struct
import sys
import tty
import termios

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
    # MOVEMENT COMMANDS (Corrected Direction for your build)
    CMD_ENABLE = _build_packet(0x08, bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    CMD_OPEN   = _build_packet(0x0C, bytes([0x02, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    CMD_CLOSE  = _build_packet(0x0C, bytes([0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    CMD_STOP   = _build_packet(0x0C, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

    def __init__(self, port: str = "/dev/ttyACM2", baudrate: int = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.is_grabbed = False
        time.sleep(0.1)
        self._send(self.CMD_ENABLE)

    def _send(self, packet: bytes):
        self.ser.write(packet)

    def set_speed(self, speed: int):
        speed = max(100, min(9999, speed))
        data = struct.pack(">H", speed) + bytes(6)
        self._send(_build_packet(0x0D, data))
        self._send(_build_packet(0x0E, data))

    def open(self):
        self.is_grabbed = False 
        self._send(self.CMD_OPEN)
        print("\n[Gripper] Opening...")
        print("[STATUS] Grab Object = False")

    def close(self):
        self._send(self.CMD_CLOSE)
        print("\n[Gripper] Closing...")
        # Since we can't read the sensor, we wait for the motion to complete
        # and then assume the object is held.
        threading.Timer(1.5, self._simulate_grab).start()

    def _simulate_grab(self):
        self.is_grabbed = True
        print("\n[STATUS] Motion Complete -> Grab Object = True")
        print("Select Action: ", end="", flush=True)

    def stop(self):
        self._send(self.CMD_STOP)
        print("\n[Gripper] Motor Halted.")

    def disconnect(self):
        self.ser.close()


# =======================
# MAIN BRIDGE NODE
# =======================
class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.is_ready = False
        self.streaming_unlocked = False
        self.cur_x, self.cur_y, self.cur_z = 0.0, 0.0, 0.0
        self.jog_step = 0.01
        
        # 1. Initialize Gripper
        self.gripper_state = 0 # 0=Closed, 1=Open
        self.gripper = None
        try:
            self.gripper = Gripper('/dev/gripper_gantry', 115200)
            self.gripper.set_speed(1200)
            self.get_logger().info("Gripper connected on /dev/ttyACM2.")
        except Exception as e:
            self.get_logger().error(f"Gripper Error: {e}")

        # 2. Initialize Gantry Arduino
        try:
            self.ser = serial.Serial('/dev/arduino_gantry', 115200, timeout=0.1)
            self.get_logger().info("Gantry Serial connection established.")
            time.sleep(2.0)
            threading.Thread(target=self.read_serial, daemon=True).start()
            threading.Thread(target=self.user_interface, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"Gantry Serial Error: {e}")

        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.listener_callback, 10
        )

    def read_serial(self):
        """Listens to the Gantry Arduino."""
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    if "HOMING_COMPLETE" in line: 
                        self.is_ready = True
                        print(f"\n[GANTRY]: System Ready (Homed)")
                    else:
                        print(f"\n[GANTRY]: {line}")
                    print("Select Action: ", end="", flush=True)

    def user_interface(self):
        while rclpy.ok():
            print("\n--- MOVEIT GANTRY BRIDGE ---")
            print("H: Home Gantry")
            print("J: Jogging Mode (WASD+QE)")
            print("C: Toggle Gripper")
            print("S: EMERGENCY STOP")
            print("R: ENABLE MOVEIT STREAMING")
            choice = input("Select Action: ").upper()

            if choice == 'H': 
                self.streaming_unlocked = False
                self.ser.write(b'H\n')
            elif choice == 'J': 
                if self.is_ready: self.run_jogging()
                else: print("Home first!")
            elif choice == 'C':
                if self.gripper:
                    if self.gripper_state == 0:
                        self.gripper.open()
                        self.gripper_state = 1
                    else:
                        self.gripper.close()
                        self.gripper_state = 0
                else:
                    print("Error: Gripper not connected!")
            elif choice == 'S': 
                self.ser.write(b'S\n')
                if self.gripper: self.gripper.stop()
                self.is_ready = False
                self.streaming_unlocked = False
            elif choice == 'R':
                if self.is_ready: 
                    self.streaming_unlocked = True
                    print(">>> NOW FOLLOWING MOVEIT TRAJECTORIES <<<")
                else: 
                    print("Error: Home 'H' first!")

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            return sys.stdin.read(1)
        finally: 
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run_jogging(self):
        print("\n--- JOGGING MODE (WASD / QE | 'C' Toggle Gripper) | ESC to EXIT ---")
        while True:
            k = self.get_key().lower()
            if k == '\x1b': break 
            
            if k == 'w': self.cur_y = min(0.360, self.cur_y + self.jog_step)
            elif k == 's': self.cur_y = max(0.0, self.cur_y - self.jog_step)
            elif k == 'a': self.cur_x = max(0.0, self.cur_x - self.jog_step)
            elif k == 'd': self.cur_x = min(0.330, self.cur_x + self.jog_step)
            elif k == 'q': self.cur_z = max(0.0, self.cur_z - self.jog_step)
            elif k == 'e': self.cur_z = min(0.150, self.cur_z + self.jog_step)
            elif k == 'c': 
                if self.gripper_state == 0:
                    self.gripper.open()
                    self.gripper_state = 1
                else:
                    self.gripper.close()
                    self.gripper_state = 0
                continue
            
            cmd = f"X{self.cur_x:.4f}Y{self.cur_y:.4f}Z{self.cur_z:.4f}\n"
            self.ser.write(cmd.encode())
            print(f"JOG POS: {cmd.strip()}    ", end='\r')

    def listener_callback(self, msg):
        if self.is_ready and self.streaming_unlocked:
            try:
                idx_x = msg.name.index('x_axis_joint')
                idx_y = msg.name.index('y_axis_joint')
                idx_z = msg.name.index('z_axis_lift_joint')
                x, y, z = msg.position[idx_x], msg.position[idx_y], msg.position[idx_z]
                command = f"X{x:.4f}Y{y:.4f}Z{z:.4f}\n"
                self.ser.write(command.encode('utf-8'))
            except (ValueError, IndexError):
                pass

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    if node.gripper: node.gripper.disconnect()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()