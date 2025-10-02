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

        self.current_command = ""  # Track what’s being held

        self.get_logger().info("Keyboard teleop started. Hold WASD keys to drive. Release to stop. Press ESC to quit.")

        # Start listening for key events
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char in self.key_mapping:
                command = self.key_mapping[key.char]
                if command != self.current_command:  # only update if changed
                    self.current_command = command
                    self.publish_command(command)
        except AttributeError:
            if key == keyboard.Key.esc:
                self.get_logger().info("ESC pressed. Stopping teleop.")
                rclpy.shutdown()

    def on_release(self, key):
        try:
            if key.char in self.key_mapping:
                # Key released → send blank string
                if self.current_command != "":
                    self.current_command = ""
                    self.publish_command("")
        except AttributeError:
            pass

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        if command == "":
            self.get_logger().info("Published stop command (blank)")
        else:
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
