#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

class ZedOdomTransformer : public rclcpp::Node
{
public:
  ZedOdomTransformer() : Node("odom_transformer"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), tf_broadcaster_(this)
  {
    this->declare_parameter<bool>("publish_tf", false);
    this->declare_parameter<std::string>("target_odom_topic", "odom");   // will be overwritten by launch file

    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    converting_odom_topic_ = this->get_parameter("target_odom_topic").as_string();

    transformed_odom_topic_ = "odom_converted";
    converted_odom_parent_frame_ = "odom";
    converted_odom_child_frame_ = "base_link";

    RCLCPP_INFO(this->get_logger(), "This node converts %s to %s", converting_odom_topic_.c_str(), transformed_odom_topic_.c_str());
    got_baselink_to_cameralink_ = false;
    

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        converting_odom_topic_, 10, std::bind(&ZedOdomTransformer::odom_callback, this, _1));

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(transformed_odom_topic_, 30);
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

  bool publish_tf_;
  std::string converting_odom_topic_;
  std::string transformed_odom_topic_;
  std::string converted_odom_parent_frame_;
  std::string converted_odom_child_frame_;

//   geometry_msgs::msg::TransformStamped tf_baselink_to_camera_link_;
  Eigen::Matrix4d T_baselink_to_camera_link_; 
  Eigen::Matrix4d T_camera_link_to_base_link_; 
  bool got_baselink_to_cameralink_ = false;

  Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::Point &translation, const geometry_msgs::msg::Quaternion &rotation)
  {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
    tf2::Matrix3x3 m(q);

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        T(i, j) = m[i][j];
      }
    }

    T(0, 3) = translation.x;
    T(1, 3) = translation.y;
    T(2, 3) = translation.z;

    return T;
  }

  Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::Vector3 &translation,
                                 const geometry_msgs::msg::Quaternion &rotation)
    {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
    tf2::Matrix3x3 m(q);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
        T(i, j) = m[i][j];
        }
    }

    T(0, 3) = translation.x;
    T(1, 3) = translation.y;
    T(2, 3) = translation.z;

    return T;
    }


  void matrixToTransform(const Eigen::Matrix4d &matrix, geometry_msgs::msg::Point &translation, geometry_msgs::msg::Quaternion &rotation)
  {
    translation.x = matrix(0, 3);
    translation.y = matrix(1, 3);
    translation.z = matrix(2, 3);

    Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0);
    tf2::Matrix3x3 tf_m(rot(0, 0), rot(0, 1), rot(0, 2),
                        rot(1, 0), rot(1, 1), rot(1, 2),
                        rot(2, 0), rot(2, 1), rot(2, 2));

    tf2::Quaternion q;
    tf_m.getRotation(q);

    rotation.x = q.x();
    rotation.y = q.y();
    rotation.z = q.z();
    rotation.w = q.w();
  }

  void setBaselinkToCameralinkTransformMat()
  {
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(
          converted_odom_child_frame_,  // target frame   "base_link"
          "camera_0_camera_link",          // source frame   "camera_0_camera_link"
          tf2::TimePointZero);
        //   tf_baselink_to_camera_link_ = transform;
          T_baselink_to_camera_link_= transformToMatrix(transform.transform.translation, transform.transform.rotation);
        RCLCPP_INFO(this->get_logger(), "Got transform from %s to %s", converted_odom_child_frame_.c_str(), "camera_0_camera_link");
        RCLCPP_INFO(this->get_logger(), "Translation: x=%f, y=%f, z=%f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
          T_camera_link_to_base_link_ = T_baselink_to_camera_link_.inverse();
        got_baselink_to_cameralink_ = true;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
      return;
    }
  }

  void setBaselinkToCameralinkTransformMat(std::string source_frame_id)
  {
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(
          converted_odom_child_frame_,  // target frame   "base_link"
          source_frame_id,          // source frame   "camera_0_camera_link"
          tf2::TimePointZero);
        //   tf_baselink_to_camera_link_ = transform;
          T_baselink_to_camera_link_= transformToMatrix(transform.transform.translation, transform.transform.rotation);
        RCLCPP_INFO(this->get_logger(), "Got transform from %s to %s", converted_odom_child_frame_.c_str(), source_frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "Translation: x=%f, y=%f, z=%f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
          T_camera_link_to_base_link_ = T_baselink_to_camera_link_.inverse();
        got_baselink_to_cameralink_ = true;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
      return;
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg)
    {
      RCLCPP_WARN(this->get_logger(), "No odometry message received");
      return;
    }
    // msg->child_frame_id = converted_odom_child_frame_;
    if(!got_baselink_to_cameralink_)
    {
        setBaselinkToCameralinkTransformMat(msg->child_frame_id);
        if(!got_baselink_to_cameralink_)
        {
            RCLCPP_WARN(this->get_logger(), "No baselink to cameralink transform available yet");
            return;
        }
    }    

    auto T_odomZed_cam = transformToMatrix(msg->pose.pose.position, msg->pose.pose.orientation);

    Eigen::Matrix4d T_odom_base = T_baselink_to_camera_link_ * T_odomZed_cam * T_camera_link_to_base_link_;

    geometry_msgs::msg::Point final_translation;
    geometry_msgs::msg::Quaternion final_rotation;
    matrixToTransform(T_odom_base, final_translation, final_rotation);

    nav_msgs::msg::Odometry transformed_odom;
    transformed_odom.header = msg->header;
    transformed_odom.header.frame_id = converted_odom_parent_frame_;
    transformed_odom.child_frame_id = converted_odom_child_frame_;

    transformed_odom.pose.pose.position = final_translation;
    transformed_odom.pose.pose.orientation = final_rotation;
    transformed_odom.pose.covariance = msg->pose.covariance;

    publisher_->publish(transformed_odom);

    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = this->now();
      tf_msg.header.frame_id = converted_odom_parent_frame_;
      tf_msg.child_frame_id = converted_odom_child_frame_;

      tf_msg.transform.translation.x = final_translation.x;
      tf_msg.transform.translation.y = final_translation.y;
      tf_msg.transform.translation.z = final_translation.z;

      tf_msg.transform.rotation = final_rotation;
      tf_broadcaster_.sendTransform(tf_msg);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ZedOdomTransformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
