#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius: %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation: %f" % self.wheel_separation_)

        # joystick publishes to bumperbot_controller/cmd_vel, this node subscribes to this topic.
        self.vel_cmd_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", 
                                                   self.velcallback, 10)
        
        # this node publishes wheel velocities to simple_velocity_controller/commands
        self.wheel_speed_pubs_ = self.create_publisher(Float64MultiArray, 
                                                       "simple_velocity_controller/commands", 10)
        
        self.speed_conversion_ = np.array([
                                    [self.wheel_radius_/2 , self.wheel_radius_/2],
                                    [self.wheel_radius_/self.wheel_separation_ , -self.wheel_radius_/self.wheel_separation_]
                                ])
        self.get_logger().info("Conversion Matrix is %s" % self.speed_conversion_)

    def velcallback(self, msg):
        robot_speed_ = np.array([
                                [msg.twist.linear.x],
                                [msg.twist.angular.z]
                            ])
        
        wheel_speed_ = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed_)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed_[1, 0], wheel_speed_[0, 0]]
        self.wheel_speed_pubs_.publish(wheel_speed_msg)

def main(args =  None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    try:
        rclpy.spin(simple_controller)
    except KeyboardInterrupt:
        pass
    finally:
        simple_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


        
