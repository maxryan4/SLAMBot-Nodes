import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher_ = self.create_publisher(String, 'motor_command', 10)

        # Key mapping: WASD → direction
        self.key_mapping = {
            'w': 'forward',
            's': 'backward',
            'a': 'left',
            'd': 'right',
        }

        self.get_logger().info("Keyboard teleop started. Use WASD keys to drive. Press ESC to quit.")

        # Start listening for key events
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char in self.key_mapping:
                command = self.key_mapping[key.char]
                self.publish_command(command)
        except AttributeError:
            # Handle special keys (e.g., ESC to stop)
            if key == keyboard.Key.esc:
                self.get_logger().info("ESC pressed. Stopping teleop.")
                rclpy.shutdown()

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
