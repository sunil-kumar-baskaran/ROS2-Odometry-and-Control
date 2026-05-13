import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('Simple_Publisher')
        self.pub_ = self.create_publisher(String, "chatter", 10)

        self.counter_ = 0
        self.frequency_ = 1.0

        self.get_logger().info("Publishing to Topic %d Hz"  % self.frequency_)
        self.create_timer(self.frequency_, self.timercallback)

    def timercallback(self):
        msg =  String()
        msg.data = "HI ROS2. YOU ARE FRIEND. Counter: %d" % self.counter_
        self.pub_.publish(msg)
        self.counter_ += 1

def main():
    rclpy.init()
    simple_publisher_obj = SimplePublisher()
    rclpy.spin(simple_publisher_obj)
    simple_publisher_obj.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
