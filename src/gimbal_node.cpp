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

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>


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
  geometry_msgs::msg::Quaternion::ConstPtr imuConst;
  geometry_msgs::msg::Quaternion::Ptr imu(new geometry_msgs::msg::Quaternion);

  if (!(parent_frame_ == imu_msg->header.frame_id))
  {
    try
    {
      if (tf2_->canTransform(parent_frame_, imu_msg->header.frame_id, imu_msg->header.stamp,
                                rclcpp::Duration::from_seconds(transform_tolerance_)))
      {
        transform = tf2_->lookupTransform(parent_frame_, imu_msg->header.frame_id, imu_msg->header.stamp);
        tf2::doTransform(imu_msg->orientation, *imu, transform);
        imuConst = imu;
      }
      else
      {
        RCLCPP_WARN_STREAM(get_logger(), "Imu_in is waiting to transform cloud from " << imu_msg->header.frame_id << " to "
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
    imuConst = std::make_shared<const geometry_msgs::msg::Quaternion>(imu_msg->orientation);
  }

  // Extract roll, pitch, and yaw
  tf2::Quaternion quat;
  tf2::convert(*imuConst, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  quat.setRPY(-roll,-pitch, 0.0);
  
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