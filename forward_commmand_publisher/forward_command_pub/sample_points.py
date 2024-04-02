import rclpy
from rclpy.node import Node
from math import sin, cos, pi

from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from sensor_msgs.msg import JointState


class CommandPublisher(Node):

    def __init__(self):
        super().__init__('forward_command_pub')
        self.names = ["xz1_servo", "yz1_servo", "xz2_servo", "yz2_servo"]
        self.sample_points = []

    def create_sample_points(self, positions):
        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "motor positions"
        msg.layout.dim[0].size = len(positions)
        msg.layout.dim[0].stride = 1
        msg.data = positions
        return msg

    def get_motor_positions(self):
        positions = []
        for name in self.names:
            position = float(input(f"Enter position for motor {name}: "))
            position = position /180 * pi
            positions.append(position)
        return positions


def main(args=None):
    # Initialize ros2
    rclpy.init(args=args)

    # Crate a node
    command_publisher = CommandPublisher()
    command_publisher.get_logger().info('Crated Node')

    # create a publisher to the forward command topic
    publisher = command_publisher.create_publisher(Float64MultiArray, '/forward_command_controller/commands', 10)

    # We need to wait a bit to give the publisher time to register, or we could publish and close the node too fast
    while publisher.get_subscription_count() == 0:
        command_publisher.get_logger().info('Waiting for subscriber', throttle_duration_sec=1)

    command_publisher.get_logger().info('Found %d subscriber(s)' % publisher.get_subscription_count())
    
    while True:
        positions = command_publisher.get_motor_positions()
        msg = command_publisher.create_sample_points(positions)
        publisher.publish(msg)
            # publish the message
        command_publisher.get_logger().info('command sent! Add next sample point or type e to exit!')
        next_sample = input("Type 'e' to exit or press Enter to add the next sample point: ")
        if next_sample.lower() == 'e':
            break


    # Destroy the node explicitly
    command_publisher.get_logger().info('Did my Job. Goodbye!')
    command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
