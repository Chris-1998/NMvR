import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher1 = self.create_publisher(String, 'map', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        file_name = open("/home/nmvr/dev_ws/src/zadanieNMvR/zadanieNMvR/map.csv")
        map = list(csv.reader(file_name, delimiter=','))
        msg1 = String()
        msg1.data = str(map)
        self.publisher1.publish(msg1)
        self.get_logger().info('Publishing: "%s"' % msg1.data)
        self.i += 1
        print(self.i)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
