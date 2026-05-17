import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import  StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse


from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform

class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__('Simple_TF_Kinematics')

        # Simulation: base footprint moves away from odom
        self.x_increment_ = 0.05
        self.last_x = 0.0 # Initialising to 0

        # Simulation: base footprint rototes w.r.t. odom
        self.rotations_counter_ = 0
        self.last_rotation_ = quaternion_from_euler(0, 0, 0) #Initialising to 0 rads
        self.rotation_increment_ = quaternion_from_euler(0, 0, 0.05) # Incrementing by 0.05 rads about z-axis

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.static_transform_broadcaster_ =  StaticTransformBroadcaster(self)
        self.dynamic_transform_broadcaster_ = TransformBroadcaster(self)
        
        self.static_transform_stamped_= TransformStamped()
        self.dynamic_transform_stamped_ = TransformStamped()

        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped_.header.frame_id = "base_footprint"
        self.static_transform_stamped_.child_frame_id = "top_frame"
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0

        self.static_transform_broadcaster_.sendTransform(self.static_transform_stamped_)

        self.get_logger().info("Publishing Static & Dynamic TF between %s and %s" 
                               % (self.static_transform_stamped_.header.frame_id, self.static_transform_stamped_.child_frame_id))
        
        self.dynamic_tf_timer_ = self.create_timer(0.1, self.timerCallback)

        self.get_transform_server_ = self.create_service(GetTransform, "get_transfomr", self.gettransformCallback)

    def timerCallback(self):
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        self.dynamic_transform_stamped_.child_frame_id = "base_footprint"
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0
        q = quaternion_multiply(self.last_rotation_, self.rotation_increment_)
        self.dynamic_transform_stamped_.transform.rotation.x = q[0]
        self.dynamic_transform_stamped_.transform.rotation.y = q[1]
        self.dynamic_transform_stamped_.transform.rotation.z = q[2]
        self.dynamic_transform_stamped_.transform.rotation.w = q[3]

        self.dynamic_transform_broadcaster_.sendTransform(self.dynamic_transform_stamped_)

        self.last_x = self.dynamic_transform_stamped_.transform.translation.x
        self.rotations_counter_ += 1
        self.last_rotation_ = q 

        if self.rotations_counter_ >= 50: # after every 50 tfs are published (note tfs are published at every 0.1secs, which means after 0.1 sec x 50 increments = 5 secs)
            self.rotation_increment_ = quaternion_inverse(self.rotation_increment_) # invert the rotation by iversing the increment
            self.rotations_counter_ = 0  #reset the counter to 0 to wait for next 50 tfs publishes in order to invert the rotation the other way around


    def gettransformCallback(self, request, response):
        self.get_logger().info("Received frames as request: %s & %s" % (request.frame_id, request.child_frame_id))
        requested_tf_content_ =  TransformStamped()
        try:
            requested_tf_content_ =  self.tf_buffer_.lookup_transform(request.frame_id, request.child_frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error("Server cannot output response TF for given/requested frames %s & %s" % (request.frame_id, request.child_frame_id))
            response.success = False
        response.frames_transform =  requested_tf_content_
        response.success =  True
        return response


def main():
    rclpy.init()
    simple_kinematics_obj = SimpleTfKinematics()
    rclpy.spin(simple_kinematics_obj)
    simple_kinematics_obj.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# TransformListener listens to the /tf and /tf_static topics.
# Buffer stores all the transforms received.
# lookup_transform() asks the buffer for a specific transform.
'''
TransformListener itself does not directly give you transform data. Its job is only to subscribe and continuously receive TF messages from the network.

The Buffer is needed because transforms are time-dependent. Robots move continuously, so TF2 must store many transforms over time. 
The buffer acts like a history database of transforms.

TransformListener receives transforms
It automatically inserts them into tf_buffer

ookup_transform() is the function that actually gives you the transform data, but it gets that data from the Buffer, not directly from TransformListener.
'''