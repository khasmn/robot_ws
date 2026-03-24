import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import threading
import struct 
from std_msgs.msg import String, Bool

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

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200, logger_callback=None):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.log = logger_callback if logger_callback else print
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
        self.log(f"[Gripper] Opening for {duration} seconds...")
        threading.Timer(duration, self.stop).start()

    def close(self, duration=1.5):
        self._send(self.CMD_CLOSE)
        self.log(f"[Gripper] Closing for {duration} seconds...")
        threading.Timer(duration, self.stop).start()

    def stop(self):
        self._send(self.CMD_STOP)
        self.log("[Gripper] Motor Halted.")

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
        self.ser = None

        # ROS Publishers & Subscribers for Web UI
        self.ui_sub = self.create_subscription(String, '/web_ui_cmd', self.web_ui_callback, 10)
        self.status_pub = self.create_publisher(String, '/gantry_status', 10)
        self.log_pub = self.create_publisher(String, '/web_log', 10)

        self.subscription = self.create_subscription(
            JointState, 
            'joint_states', 
            self.listener_callback, 
            10)

        # Connect Gantry
        try:
            self.ser = serial.Serial('/dev/arduino_gantry', 115200, timeout=0.1)
            self.get_logger().info("Gantry Serial connection established.")
            time.sleep(2.0)
            threading.Thread(target=self.read_serial, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"Gantry Serial Error: {e}")

        try:
            self.gripper = Gripper(port="/dev/ttyACM0", logger_callback=self.web_log)
            self.web_log("[SYSTEM] Gripper Connected.")
        except Exception as e:
            self.get_logger().warning("Gripper not found or failed to connect.")
        
    def web_log(self, text):
        print(text)
        msg = String()
        msg.data = str(text)
        self.log_pub.publish(msg)

    def read_serial(self):
        while rclpy.ok():
            if self.ser and self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.web_log(f"[ARDUINO]: {line}")
                    if "HOMING_COMPLETE" in line:
                        self.is_ready = True
                        msg = String()
                        msg.data = "HOMED"
                        self.status_pub.publish(msg)

    def listener_callback(self, msg):
        if self.is_ready and self.streaming_unlocked:
            try:
                idx_x = msg.name.index('x_axis_joint')
                idx_y = msg.name.index('y_axis_joint')
                idx_z = msg.name.index('z_axis_lift_joint')
                x, y, z = msg.position[idx_x], msg.position[idx_y], msg.position[idx_z]
                
                command = f"X{x:.4f}Y{y:.4f}Z{z:.4f}\n"
                if self.ser: self.ser.write(command.encode('utf-8'))
            except (ValueError, IndexError):
                pass

    def web_ui_callback(self, msg):
        cmd = msg.data.upper()
        
        if cmd == 'OPEN':
            if self.gripper: self.gripper.open()
        
        elif cmd == 'CLOSE':
            if self.gripper: self.gripper.close()
        
        elif cmd == 'HOME':
            self.streaming_unlocked = False
            if self.ser: self.ser.write(b'H\n')
            status_msg = String()
            status_msg.data = "HOMING..."
            self.status_pub.publish(status_msg)
            self.web_log("[SYSTEM] Homing sequence initiated...")
            
        elif cmd == 'STREAM_ON':
            if self.is_ready:
                self.streaming_unlocked = True
                self.web_log("[SYSTEM] >>> NOW FOLLOWING MOVEIT TRAJECTORIES <<<")
            else:
                self.web_log("[ERROR] You must Home the gantry first!")
                
        elif cmd == 'S':
            if self.ser: self.ser.write(b'S\n')
            if self.gripper: self.gripper.stop()
            self.is_ready = False
            self.streaming_unlocked = False
            self.web_log("[WARNING] EMERGENCY STOP ACTIVATED! Motors halted.")
            status_msg = String()
            status_msg.data = "STOPPED"
            self.status_pub.publish(status_msg)

        elif cmd.startswith('JOG_'):
            if not self.is_ready:
                self.web_log("[ERROR] Cannot jog: Gantry not homed!")
                return
                
            if self.streaming_unlocked:
                self.web_log("[WARNING] Stop MoveIt stream ('S') before manually jogging!")
                return

            # Send compact commands to the Arduino
            if self.ser:
                if cmd == 'JOG_X+': self.ser.write(b'JX+\n')
                elif cmd == 'JOG_X-': self.ser.write(b'JX-\n')
                elif cmd == 'JOG_Y+': self.ser.write(b'JY+\n')
                elif cmd == 'JOG_Y-': self.ser.write(b'JY-\n')
                elif cmd == 'JOG_STOP': self.ser.write(b'JS\n')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.gripper:
            node.gripper.disconnect()
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()