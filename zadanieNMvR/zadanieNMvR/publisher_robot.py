import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher2 = self.create_publisher(String, 'topic', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg2 = String()
        terminal = input('Position (x y) or Direction (R,L,U,D,UR,UL,DR,DL):')
        msg2.data = terminal
        self.publisher2.publish(msg2)
        self.get_logger().info('Publishing: "%s"' % msg2.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()