#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value 

        # Previous Values
        self.prev_left_wheel_pos = 0.0
        self.prev_right_wheel_pos = 0.0
        self.prev_time_ = self.get_clock().now()

        self.get_logger().info("Using wheel radius: %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation: %f" % self.wheel_separation_)

        # joystick publishes to bumperbot_controller/cmd_vel, this node subscribes to this topic.
        self.vel_cmd_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", 
                                                   self.velcallback, 10)
        
        # this node publishes wheel velocities to simple_velocity_controller/commands
        self.wheel_speed_pubs_ = self.create_publisher(Float64MultiArray, 
                                                       "simple_velocity_controller/commands", 10)
        
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        
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

    def jointCallback(self, msg):
        dp_left = msg.position[0] - self.prev_left_wheel_pos
        dp_right = msg.position[1] - self.prev_right_wheel_pos
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.prev_left_wheel_pos =  msg.position[0]
        self.prev_right_wheel_pos = msg.position[1]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        dt_sec =  (dt.nanoseconds / S_TO_NS)
        
        if dt_sec <= 0.0:
            self.get_logger().warn("dt is zero, skipping calculation")
            return
        
        fi_left  =  dp_left / dt_sec
        fi_right  =  dp_right / dt_sec

        linear =  (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular =  (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        self.get_logger().info("Linear: %f \n Angular: %f" % (linear, angular))
        

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


        
