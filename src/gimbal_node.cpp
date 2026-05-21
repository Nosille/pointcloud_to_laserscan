/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Open Source Robotics Foundation, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#include "pointcloud_to_laserscan/gimbal_node.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <tf2_eigen/tf2_eigen.hpp>


namespace pointcloud_to_laserscan
{
GimbalNode::GimbalNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("gimbal", options)
{
  tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
  tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  child_frame_ = this->declare_parameter("child_frame", "");
  parent_frame_ = this->declare_parameter("parent_frame", "");
  transform_tolerance_ = this->declare_parameter("transform_tolerance", 0.01);

  // ROS publishers and subscribers
  auto imu_qos = rclcpp::SensorDataQoS().keep_last(200).best_effort();  
  auto imu_topic = this->declare_parameter("imu_topic", "imu_in");
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic, imu_qos, std::bind(&GimbalNode::imuCallback, this, std::placeholders::_1));  
}

sensor_msgs::msg::Imu::SharedPtr GimbalNode::transformImu(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_raw, 
                                                   const geometry_msgs::msg::TransformStamped& transform)
{
    sensor_msgs::msg::Imu::SharedPtr imu(new sensor_msgs::msg::Imu);
    Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform);

    // Copy header
    imu->header = imu_raw->header;

    // Transform orientation
    Eigen::Quaterniond orientation(imu_raw->orientation.w, imu_raw->orientation.x, 
                                    imu_raw->orientation.y, imu_raw->orientation.z);
    Eigen::Quaterniond rotation(transform_eigen.rotation());
    Eigen::Quaterniond quat_transformed = orientation * rotation.inverse();

    imu->orientation.w = quat_transformed.w();
    imu->orientation.x = quat_transformed.x();
    imu->orientation.y = quat_transformed.y();
    imu->orientation.z = quat_transformed.z();

    // Transform angular velocity
    Eigen::Vector3d ang_vel(imu_raw->angular_velocity.x,
                            imu_raw->angular_velocity.y,
                            imu_raw->angular_velocity.z);
    Eigen::Vector3d ang_vel_transformed = transform_eigen.rotation() * ang_vel;

    imu->angular_velocity.x = ang_vel_transformed[0];
    imu->angular_velocity.y = ang_vel_transformed[1];
    imu->angular_velocity.z = ang_vel_transformed[2];

    // Transform linear acceleration (accounting for centripetal acceleration)
    Eigen::Vector3d lin_accel(imu_raw->linear_acceleration.x,
                              imu_raw->linear_acceleration.y,
                              imu_raw->linear_acceleration.z);
    Eigen::Vector3d lin_accel_transformed = transform_eigen.rotation() * lin_accel
                                            + ang_vel_transformed.cross(ang_vel_transformed.cross(-transform_eigen.translation()));

    imu->linear_acceleration.x = lin_accel_transformed[0];
    imu->linear_acceleration.y = lin_accel_transformed[1];
    imu->linear_acceleration.z = lin_accel_transformed[2];

    return imu;
}

void GimbalNode::imuCallback(const sensor_msgs::msg::Imu::ConstPtr& imu_msg)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "IMU Callback");
  
  if(child_frame_ == "") 
  {
    RCLCPP_INFO_STREAM(get_logger(), "child_frame not specified!");
    return;
  }

  // transform imu to parent frame
  geometry_msgs::msg::TransformStamped transform;
  sensor_msgs::msg::Imu::SharedPtr transformed_msg;
  if (!(parent_frame_ == imu_msg->header.frame_id))
  {
    try
    {
      if (tf2_->canTransform(parent_frame_, imu_msg->header.frame_id, imu_msg->header.stamp,
                                rclcpp::Duration::from_seconds(transform_tolerance_)))
      {
        transform = tf2_->lookupTransform(parent_frame_, imu_msg->header.frame_id, imu_msg->header.stamp);
        transformed_msg = transformImu(imu_msg, transform);
      }
      else
      {
        RCLCPP_WARN_STREAM(get_logger(), "Imu_in is waiting to transform from " << imu_msg->header.frame_id << " to "
                                                                << parent_frame_ << ".");
        return;
      }
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }
  else
  {
     transformed_msg = std::make_shared<sensor_msgs::msg::Imu>(sensor_msgs::msg::Imu(*imu_msg));
  }

  // Extract roll, pitch, and yaw
  tf2::Quaternion quat;
  tf2::convert(transformed_msg->orientation, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  quat.setRPY(-roll,-pitch, 0.0);
  // RCLCPP_INFO_STREAM(get_logger(), "roll: "  << roll);
  // RCLCPP_INFO_STREAM(get_logger(), "pitch: " << pitch); 
  
  //Publish transform from parent to child frame id's
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = imu_msg->header.stamp;
  tf_msg.header.frame_id = parent_frame_;
  tf_msg.child_frame_id  = child_frame_;
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf2::convert(quat, tf_msg.transform.rotation);

  tf2_broadcaster_->sendTransform(tf_msg);
}
}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::GimbalNode)