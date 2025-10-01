import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

# GPIO pins
LEFT_FORWARD = 5
LEFT_BACKWARD = 6
RIGHT_FORWARD = 13
RIGHT_BACKWARD = 26

# Motor setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(LEFT_FORWARD, GPIO.OUT)
GPIO.setup(LEFT_BACKWARD, GPIO.OUT)
GPIO.setup(RIGHT_FORWARD, GPIO.OUT)
GPIO.setup(RIGHT_BACKWARD, GPIO.OUT)

# Helper functions
def stop_motors():
    GPIO.output(LEFT_FORWARD, False)
    GPIO.output(LEFT_BACKWARD, False)
    GPIO.output(RIGHT_FORWARD, False)
    GPIO.output(RIGHT_BACKWARD, False)

def move_forward():
    GPIO.output(LEFT_FORWARD, True)
    GPIO.output(LEFT_BACKWARD, False)
    GPIO.output(RIGHT_FORWARD, True)
    GPIO.output(RIGHT_BACKWARD, False)

def move_backward():
    GPIO.output(LEFT_FORWARD, False)
    GPIO.output(LEFT_BACKWARD, True)
    GPIO.output(RIGHT_FORWARD, False)
    GPIO.output(RIGHT_BACKWARD, True)

def turn_left():
    GPIO.output(LEFT_FORWARD, False)
    GPIO.output(LEFT_BACKWARD, True)
    GPIO.output(RIGHT_FORWARD, True)
    GPIO.output(RIGHT_BACKWARD, False)

def turn_right():
    GPIO.output(LEFT_FORWARD, True)
    GPIO.output(LEFT_BACKWARD, False)
    GPIO.output(RIGHT_FORWARD, False)
    GPIO.output(RIGHT_BACKWARD, True)

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
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
