import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio
import time

# GPIO pins (BCM mode)
LEFT_FORWARD = 5
LEFT_BACKWARD = 6
RIGHT_FORWARD = 13
RIGHT_BACKWARD = 26

# Open GPIO chip
CHIP = lgpio.gpiochip_open(0)

# Setup pins as outputs
for pin in [LEFT_FORWARD, LEFT_BACKWARD, RIGHT_FORWARD, RIGHT_BACKWARD]:
    lgpio.gpio_claim_output(CHIP, pin)

# Helper functions
def stop_motors():
    lgpio.gpio_write(CHIP, LEFT_FORWARD, 0)
    lgpio.gpio_write(CHIP, LEFT_BACKWARD, 0)
    lgpio.gpio_write(CHIP, RIGHT_FORWARD, 0)
    lgpio.gpio_write(CHIP, RIGHT_BACKWARD, 0)

def move_forward():
    lgpio.gpio_write(CHIP, LEFT_FORWARD, 1)
    lgpio.gpio_write(CHIP, LEFT_BACKWARD, 0)
    lgpio.gpio_write(CHIP, RIGHT_FORWARD, 0)
    lgpio.gpio_write(CHIP, RIGHT_BACKWARD, 1)

def move_backward():
    lgpio.gpio_write(CHIP, LEFT_FORWARD, 0)
    lgpio.gpio_write(CHIP, LEFT_BACKWARD, 1)
    lgpio.gpio_write(CHIP, RIGHT_FORWARD, 1)
    lgpio.gpio_write(CHIP, RIGHT_BACKWARD, 0)

def turn_left():
    lgpio.gpio_write(CHIP, LEFT_FORWARD, 0)
    lgpio.gpio_write(CHIP, LEFT_BACKWARD, 1)
    lgpio.gpio_write(CHIP, RIGHT_FORWARD, 0)
    lgpio.gpio_write(CHIP, RIGHT_BACKWARD, 1)

def turn_right():
    lgpio.gpio_write(CHIP, LEFT_FORWARD, 1)
    lgpio.gpio_write(CHIP, LEFT_BACKWARD, 0)
    lgpio.gpio_write(CHIP, RIGHT_FORWARD, 1)
    lgpio.gpio_write(CHIP, RIGHT_BACKWARD, 0)

# ROS 2 Node
class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.command_callback,
            10
        )
        self.get_logger().info('Motor node started, waiting for commands...')

    def command_callback(self, msg: String):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        stop_motors()  # Stop before executing new command

        if command == 'forward':
            move_forward()
        elif command == 'backward':
            move_backward()
        elif command == 'left':
            turn_left()
        elif command == 'right':
            turn_right()
        else:
            stop_motors()

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        lgpio.gpiochip_close(CHIP)   # release GPIOs
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
