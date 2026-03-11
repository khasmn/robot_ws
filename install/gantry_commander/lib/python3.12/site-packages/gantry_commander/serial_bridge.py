import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import threading

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.is_ready = False
        self.streaming_unlocked = False
        
        try:
            # Arduino port
            self.ser = serial.Serial('/dev/arduino_gantry', 115200, timeout=0.1)
            self.get_logger().info("Serial connection established.")
            time.sleep(2.0)
            threading.Thread(target=self.read_serial, daemon=True).start()
            threading.Thread(target=self.user_interface, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")

        # Subscribing to MoveIt joint state output
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
            print("S: EMERGENCY STOP")
            print("R: ENABLE MOVEIT STREAMING")
            choice = input("Select Action: ").upper()

            if choice == 'H':
                self.streaming_unlocked = False
                self.ser.write(b'H\n')
            elif choice == 'T':
                self.ser.write(b'T\n')
            elif choice == 'S':
                self.ser.write(b'S\n')
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
                # Find names in your URDF
                idx_x = msg.name.index('x_axis_joint')
                idx_y = msg.name.index('y_axis_joint')
                idx_z = msg.name.index('z_axis_lift_joint')

                x = msg.position[idx_x]
                y = msg.position[idx_y]
                z = msg.position[idx_z]
                
                command = f"X{x:.4f}Y{y:.4f}Z{z:.4f}\n"
                self.ser.write(command.encode('utf-8'))
                
                # THIS FOR DEBUGGING:
                print(f"Sent to Arduino: {command.strip()}", end='\r')

            except (ValueError, IndexError) as e:
                print(f"Waiting for joints... Error: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()