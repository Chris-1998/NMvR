import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublishe(Node):

    def __init__(self):
        super().__init__('minimal_publishe')
        self.publisher2 = self.create_publisher(String, 'stena', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg11 = String()
        terminal = input('Position (x y): ')
        msg11.data = terminal
        self.publisher2.publish(msg11)
        self.get_logger().info('Publishing: "%s"' % msg11.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublishe()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()