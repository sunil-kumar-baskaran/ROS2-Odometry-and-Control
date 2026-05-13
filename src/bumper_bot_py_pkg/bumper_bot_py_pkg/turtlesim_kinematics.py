import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math

class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_kinematics")

        self.turtle1_pose_sub_ = self.create_subscription(Pose, "turtle1/pose", self.turtle1PoseCallback, 10)
        self.turtle_pose_sub_ = self.create_subscription(Pose, "turtle2/pose",  self.turtle2PoseCallback, 10)

        self.last_turtle1_pose_value_ =  Pose()
        self.last_turtle2_pose_value_ =  Pose()

    def turtle1PoseCallback(self, msg):
        self.last_turtle1_pose_value_ = msg

    def turtle2PoseCallback(self, msg):
        self.last_turtle2_pose_value_ = msg
        
        Tx = self.last_turtle2_pose_value_.x - self.last_turtle1_pose_value_.x
        Ty = self.last_turtle2_pose_value_.y - self.last_turtle1_pose_value_.y

        theta_deg_ = self.last_turtle2_pose_value_.theta - self.last_turtle1_pose_value_.theta
        theta_rad_ = 180 * theta_deg_/ 3.14

        self.get_logger().info(""" \n
                               Translation vector T between turtle1 & turtle 2 \n
                               Tx: %f \n
                               Ty: %f \n
                               Rotation matrix R between turtle1 & turtle 2 \n
                               theta_deg: %f \n
                               theta_rad: %f  \n
                               |R11 R12|: |%f %f| \n
                               |R21 R22|: |%f %f|""" % (Tx, Ty, theta_deg_, theta_rad_, 
                                                        math.cos(theta_rad_), 
                                                        -math.sin(theta_rad_), 
                                                        math.sin(theta_rad_), 
                                                        math.cos(theta_rad_) ))
        
        
        

def main():
    rclpy.init()
    simple_turtlesim_kinematics = SimpleTurtlesimKinematics()
    rclpy.spin(simple_turtlesim_kinematics)
    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()