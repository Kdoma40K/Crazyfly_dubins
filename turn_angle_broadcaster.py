import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TurnAngleBroadcaster(Node):
    def __init__(self):
        super().__init__('turn_angle_broadcaster')
        self.publisher_ = self.create_publisher(Float32, 'turn_angle', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0

    def timer_callback(self):
        msg = Float32()
        msg.data = self.angle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.angle += 0.1  # increment the angle for demonstration


def main(args=None):
    rclpy.init(args=args)
    node = TurnAngleBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
