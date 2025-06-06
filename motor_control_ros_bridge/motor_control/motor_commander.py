import rclpy
from rclpy.node import Node
import serial

class MotorCommandNode(Node):
    def __init__(self):
        super().__init__('motor_command_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.get_logger().info("MotorCommandNode initialized. Type 'f' or 'r' and press Enter.")

        self.timer = self.create_timer(0.5, self.get_input_and_send)

    def get_input_and_send(self):
        try:
            user_input = input("Enter direction (f = forward, r = reverse): ").strip().lower()
            if user_input == 'f':
                self.ser.write(b'F')
                self.get_logger().info("Sent: Forward")
            elif user_input == 'r':
                self.ser.write(b'R')
                self.get_logger().info("Sent: Reverse")
            else:
                self.get_logger().info("Invalid input.")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

