import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import math
import numpy as np
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster

def euler_from_quaternion(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def transform_to_matrix(translation, rotation):
    """Convert translation and rotation to 4x4 matrix."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(rotation).as_matrix()
    T[:3, 3] = translation
    return T

def matrix_to_transform(matrix):
    """Convert 4x4 matrix to translation and rotation."""
    translation = matrix[:3, 3]
    rotation = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return translation, rotation

class OdomTransformer(Node):
    def __init__(self):
        super().__init__('odom_transformer')
        
        # Parameters
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('target_odom_topic', '/odom')

        self.publish_tf_ = self.get_parameter('publish_tf').get_parameter_value().bool_value
        
        # target_odom_topic = self.get_parameter('target_odom_topic').get_parameter_value().string_value


        self.tf_broadcaster = TransformBroadcaster(self)

        self.converting_odom_topic = self.get_parameter('target_odom_topic').get_parameter_value().string_value #'/j100_0921/sensors/camera_0/stereolabs_zed/odom' # odom_zed --> camera_link
        self.converted_odom_topic  = 'odom_converted' 
        self.converted_odom_parent_frame = 'odom'
        self.converted_odom_child_frame = 'base_link'

        self.get_logger().info(f'This node converts {self.converting_odom_topic} to {self.converted_odom_topic}')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        self.subscription = self.create_subscription(
            Odometry,
            self.converting_odom_topic,
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(Odometry, self.converted_odom_topic, 30)
        # self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.latest_odom = None
        self.previous_odom = None
        self.previous_time = None
    

    def odom_callback(self, msg):
        self.latest_odom = msg
        if self.latest_odom is None:
            self.get_logger().warning(f'{self.converting_odom_topic} is not being published.')
            return
        
        try:
            
            # odom2odomZed_tf = self.tf_buffer.lookup_transform('odom', 'odom_zed', self.get_clock().now())
            #transform = self.tf_buffer.lookup_transform('husky1_camera_link','base_link',  self.get_clock().now())
            transform = self.tf_buffer.lookup_transform(self.latest_odom.child_frame_id,self.converted_odom_child_frame,  self.get_clock().now())

            self.get_logger().debug(f'{str(transform.transform.translation.x)}, {str(transform.transform.translation.y)}, {str(transform.transform.translation.z)}')
            
            transformed_odom = Odometry()
            transformed_odom.header = self.latest_odom.header
            transformed_odom.header.frame_id = self.converted_odom_parent_frame
            transformed_odom.child_frame_id = self.converted_odom_child_frame

            roll, pitch, yaw = euler_from_quaternion(transformed_odom.pose.pose.orientation.x,transformed_odom.pose.pose.orientation.y,transformed_odom.pose.pose.orientation.z,transformed_odom.pose.pose.orientation.w)
            
            T_odomZed_cam = transform_to_matrix(
                [self.latest_odom.pose.pose.position.x , self.latest_odom.pose.pose.position.y, self.latest_odom.pose.pose.position.z], 
                [self.latest_odom.pose.pose.orientation.x, self.latest_odom.pose.pose.orientation.y, self.latest_odom.pose.pose.orientation.z, self.latest_odom.pose.pose.orientation.w]
                )
            
            T_cam_base = transform_to_matrix([transform.transform.translation.x ,transform.transform.translation.y, transform.transform.translation.z], 
                                             [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
            T_odom_odomZed = transform_to_matrix([0.252, 0.0, 0.226], [0, 0, 0, 1])    # todo: get this from the static transform publisher node                                         

            T_odomZed_base = np.dot(T_odomZed_cam, T_cam_base)
            T_odom_base = np.dot(T_odom_odomZed, T_odomZed_base)

            final_translation, final_rotation = matrix_to_transform(T_odom_base)
            
            transformed_odom.pose.pose.position.x = final_translation[0]
            transformed_odom.pose.pose.position.y = final_translation[1]
            transformed_odom.pose.pose.position.z = final_translation[2]
            transformed_odom.pose.pose.orientation.x = final_rotation[0]
            transformed_odom.pose.pose.orientation.y = final_rotation[1]
            transformed_odom.pose.pose.orientation.z = final_rotation[2]
            transformed_odom.pose.pose.orientation.w = final_rotation[3]
            transformed_odom.pose.covariance = self.latest_odom.pose.covariance
            
            # Calculate velocity
            current_time = self.get_clock().now()
            # if self.previous_odom is not None and self.previous_time is not None:
            #     dt = (current_time - self.previous_time).nanoseconds / 1e9
            #     linear_vel, angular_vel = self.calculate_velocity(transformed_odom, self.previous_odom, dt)
                
            #     # Set the calculated velocity in the twist
            #     transformed_odom.twist.twist.linear = linear_vel
            #     transformed_odom.twist.twist.angular = angular_vel
            
            tf_msg = TransformStamped()
             # Set the header
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = self.converted_odom_parent_frame
            
            # Set the child frame ID
            tf_msg.child_frame_id = self.converted_odom_child_frame 
            
            # Set the translation and rotation
            tf_msg.transform.translation.x = transformed_odom.pose.pose.position.x
            tf_msg.transform.translation.y = transformed_odom.pose.pose.position.y
            tf_msg.transform.translation.z = transformed_odom.pose.pose.position.z
            tf_msg.transform.rotation = transformed_odom.pose.pose.orientation
            
            self.publisher.publish(transformed_odom)

            if(self.publish_tf_):
                # Broadcast the transform odom-->base_link
                self.tf_broadcaster.sendTransform(tf_msg)
            
            # Update previous data
            self.previous_odom = transformed_odom
            self.previous_time = current_time
        
        except Exception as e:
            self.get_logger().warn(f'Failed to transform odometry: {str(e)}')

    # 2D 
    def calculate_velocity(self, current_odom, previous_odom, dt):
        if previous_odom is None or dt == 0:
            return Vector3(), Vector3()

        # Calculate linear velocity
        dx = current_odom.pose.pose.position.x - previous_odom.pose.pose.position.x
        dy = current_odom.pose.pose.position.y - previous_odom.pose.pose.position.y
        dz = current_odom.pose.pose.position.z - previous_odom.pose.pose.position.z

        linear_vel = Vector3()
        linear_vel.x = dx / dt
        linear_vel.y = dy / dt
        linear_vel.z = dz / dt

        # Calculate angular velocity (simplified, assumes small angle changes)
        current_yaw = self.euler_from_quaternion(current_odom.pose.pose.orientation)
        previous_yaw = self.euler_from_quaternion(previous_odom.pose.pose.orientation)
        dyaw = current_yaw - previous_yaw

        # Normalize the angle difference
        if dyaw > math.pi:
            dyaw -= 2 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2 * math.pi

        angular_vel = Vector3()
        angular_vel.z = dyaw / dt

        return linear_vel, angular_vel

    def euler_from_quaternion(self, quaternion):
        # Simple conversion from quaternion to Euler angles (yaw only)
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    # def timer_callback(self):
        if self.latest_odom is None:
            self.get_logger().info(f'{self.converting_odom_topic} is not being published.')
            return
        
        try:
            transform = self.tf_buffer.lookup_transform(self.latest_odom.child_frame_id,self.converted_odom_child_frame,  self.get_clock().now())
            # transform = self.tf_buffer.lookup_transform('husky1_camera_link','base_link',  self.get_clock().now())

            

            self.get_logger().info(f'{str(transform.transform.translation.x)}, {str(transform.transform.translation.y)}, {str(transform.transform.translation.z)}')
            
            transformed_odom = Odometry()
            transformed_odom.header = self.latest_odom.header
            transformed_odom.header.frame_id = 'map'
            transformed_odom.child_frame_id = self.converted_odom_child_frame 
            
            # Transform pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(
                self.latest_odom.pose.pose,
                transform
            )
            transformed_odom.pose.pose = transformed_pose
            
            # Calculate velocity
            current_time = self.get_clock().now()
            if self.previous_odom is not None and self.previous_time is not None:
                dt = (current_time - self.previous_time).nanoseconds / 1e9
                linear_vel, angular_vel = self.calculate_velocity(self.latest_odom, self.previous_odom, dt)
                
                # Set the calculated velocity in the twist
                transformed_odom.twist.twist.linear = linear_vel
                transformed_odom.twist.twist.angular = angular_vel
            
            self.publisher.publish(transformed_odom)
            
            # Update previous data
            self.previous_odom = self.latest_odom
            self.previous_time = current_time
        
        except Exception as e:
            self.get_logger().warn(f'Failed to transform odometry: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
