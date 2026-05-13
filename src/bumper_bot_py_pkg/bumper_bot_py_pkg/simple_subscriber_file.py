import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("Simple_subscriber")
        self.sub_ = self.create_subscription(String, "chatter",  self.msgCallback, 10)

    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)


def main():
    rclpy.init()
    simple_subcriber =  SimpleSubscriber()
    rclpy.spin(simple_subcriber)
    simple_subcriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
