#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler

from tf2_ros import TransformBroadcaster

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

        #Initialisations
        self.x_ = 0.0
        self.y_ =  0.0 
        self.theta_ = 0.0 
        
        self.get_logger().info("Using wheel radius: %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation: %f" % self.wheel_separation_)
        
        # joystick publishes to bumperbot_controller/cmd_vel, this node subscribes to this topic.
        self.vel_cmd_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", 
                                                   self.velcallback, 10)
        
        # this node publishes wheel velocities to simple_velocity_controller/commands
        self.wheel_speed_pubs_ = self.create_publisher(Float64MultiArray, 
                                                       "simple_velocity_controller/commands", 10)
        
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)

        # publish quaternion messages for position and orientation & linear, angular velocities calculated here
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controllers/odom", 10)
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.transform_stamped_ = TransformStamped()
        self.transform_broadcaster_ =  TransformBroadcaster(self)

        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"
        

        
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
    
        fi_left  =  dp_left / dt_sec
        fi_right  =  dp_right / dt_sec
        
        # Linear Velocity of the Robot
        linear =  (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        # Angular Velocity of the Robot
        angular =  (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_
        # Position of the Robot
        d_pos = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2 #position vector
        # Orientation of the Robot
        d_theta = angular =  (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_ # orientation angle

        self.theta_ += d_theta #accumnulatijng the orientation angle
        self.x_ += d_pos * math.cos(self.theta_) #accumulating the x component of the position vector
        self.y_ += d_pos * math.sin(self.theta_) # accumulating the y component of the postion vector

        q = quaternion_from_euler(0, 0,  self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        
        self.odom_pub_.publish(self.odom_msg_)

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        self.transform_broadcaster_.sendTransform(self.transform_stamped_)
        

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


        
