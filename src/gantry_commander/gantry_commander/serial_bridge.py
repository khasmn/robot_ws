import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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
    CMD_OPEN   = _build_packet(0x0C, bytes([0x02, 0x13, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00]))
    CMD_CLOSE  = _build_packet(0x0C, bytes([0x01, 0x13, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00])) 
    CMD_STOP   = _build_packet(0x0C, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

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
        self.gripper_state = 0
        self.gripper = None

        # --- Gripper setup ---
        try:
            self.gripper = Gripper('/dev/gripper_gantry', 115200)
            self.gripper.set_speed(800)
            self.get_logger().info("Gripper connected.")
        except Exception as e:
            self.get_logger().error(f"Gripper Error: {e}")

        # --- Arduino serial setup ---
        try:
            self.ser = serial.Serial('/dev/arduino_gantry', 115200, timeout=0.1)
            self.get_logger().info("Serial connection established.")
            time.sleep(2.0)
            self.ser.write(b'H\n')
            threading.Thread(target=self.read_serial, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")

        # --- Subscribers ---
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

        self.create_subscription(
            String,
            '/gripper_command',
            self.gripper_command_callback,
            10)

        # --- Publishers ---
        self.gripper_status_pub = self.create_publisher(String, '/gripper_status', 10)
        
        # [FIX ADDED]: We need this publisher to tell YOLO the robot is ready!
        self.robot_ready_pub = self.create_publisher(String, '/robot_ready', 10)

        self.get_logger().info("SerialBridge node ready.")

    # ------------------------------------------------------------------
    # Arduino serial reader (runs in background thread)
    # ------------------------------------------------------------------
    def read_serial(self):
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"\n[ARDUINO]: {line}")
                    # Note: Ensure your Arduino actually prints exactly "HOMING_COMPLETE"
                    if "HOMING_COMPLETE" in line:
                        threading.Thread(
                            target=self._gripper_self_test, daemon=True
                        ).start()

    # ------------------------------------------------------------------
    # Gripper self-test — runs once after homing completes
    # ------------------------------------------------------------------
    def _gripper_self_test(self):
        if self.gripper is None:
            self.get_logger().warn("Gripper self-test skipped: gripper not connected.")
            self.is_ready = True
            
            # [FIX ADDED]: Still broadcast ready so YOLO works even if gripper is unplugged
            ready_msg = String()
            ready_msg.data = 'ready'
            self.robot_ready_pub.publish(ready_msg)
            return

        self.get_logger().info("=== Gripper self-test starting ===")

        # Step 1 — Open
        self.get_logger().info("[Self-test] Step 1/3: Opening gripper...")
        self.gripper.open(duration=3.5)
        time.sleep(4.0)  

        # Step 2 — Close
        self.get_logger().info("[Self-test] Step 2/3: Closing gripper...")
        self.gripper.close(duration=3.5)
        time.sleep(4.0)

        # Step 3 — Open again (leave ready to grab)
        self.get_logger().info("[Self-test] Step 3/3: Opening gripper to standby position...")
        self.gripper.open(duration=3.5)
        time.sleep(4.0)

        self.get_logger().info("=== Gripper self-test complete. Robot is READY. ===")
        self.is_ready = True   

        # [FIX ADDED]: Actually publish the message to unlock YOLO!
        ready_msg = String()
        ready_msg.data = 'ready'
        self.robot_ready_pub.publish(ready_msg)
        self.get_logger().info("Broadcasted /robot_ready signal to YOLO.")

    # ------------------------------------------------------------------
    # Joint-state → Arduino bridge
    # ------------------------------------------------------------------
    def listener_callback(self, msg):
        if self.is_ready:
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

    # ------------------------------------------------------------------
    # Gripper command subscriber  (/gripper_command)
    # ------------------------------------------------------------------
    def gripper_command_callback(self, msg: String):
        if self.gripper is None:
            self.get_logger().error("Gripper command received but gripper is not connected.")
            return

        cmd = msg.data.strip().lower()
        self.get_logger().info(f"Gripper command received: '{cmd}'")

        if cmd == 'close':
            self.gripper.close(duration=3.5)
            threading.Timer(3.6, self._publish_gripper_status, args=('closed',)).start()

        elif cmd == 'open':
            self.gripper.open(duration=3.5)
            threading.Timer(3.6, self._publish_gripper_status, args=('opened',)).start()

        elif cmd == 'stop':
            self.gripper.stop()
            self._publish_gripper_status('stopped')

        else:
            self.get_logger().warn(f"Unknown gripper command: '{cmd}'. Use 'open', 'close', or 'stop'.")

    # ------------------------------------------------------------------
    # Gripper status publisher  (/gripper_status)
    # ------------------------------------------------------------------
    def _publish_gripper_status(self, status: str):
        out = String()
        out.data = status
        self.gripper_status_pub.publish(out)
        self.get_logger().info(f"Gripper status published: '{status}'")


# =======================
# ENTRY POINT
# =======================
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