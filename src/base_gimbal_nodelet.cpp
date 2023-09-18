/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Eurotec, Netherlands
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

/*
 * Author: Charles Ellison
 */

#include <pointcloud_to_laserscan/base_gimbal_nodelet.h>
#include <sensor_msgs/Imu.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace pointcloud_to_laserscan
{
BaseGimbalNodelet::BaseGimbalNodelet()
: tfBuffer_(ros::Duration(300))
, tfListener_(tfBuffer_)
{
}

void BaseGimbalNodelet::onInit()
{
  nh_ = getMTNodeHandle();
  private_nh_ = getMTPrivateNodeHandle();

  tfBuffer_.setUsingDedicatedThread(true);

  private_nh_.param<std::string>("child_frame", child_frame_, "");
  ROS_INFO_STREAM("child_frame: " << child_frame_);
  private_nh_.param<std::string>("parent_frame", parent_frame_, "");
  ROS_INFO_STREAM("parent_frame: " << parent_frame_);
  private_nh_.param<double>("transform_tolerance", transform_tolerance_, 0.01);

  // ROS publishers and subscribers
  sub_ = nh_.subscribe<sensor_msgs::Imu>("imu_in", 100, &BaseGimbalNodelet::imuCallback, this);
}

void BaseGimbalNodelet::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
  ROS_DEBUG_STREAM("IMU Callback");
  
  if(child_frame_ == "") 
  {
    ROS_INFO_STREAM("child_frame not specified!");
    return;
  }

  // transform imu to parent frame
  geometry_msgs::TransformStamped transform;
  geometry_msgs::QuaternionConstPtr imuConst;
  geometry_msgs::QuaternionPtr imu(new geometry_msgs::Quaternion);

  if (!(parent_frame_ == imu_msg->header.frame_id))
  {
    try
    {
      if (tfBuffer_.canTransform(parent_frame_, imu_msg->header.frame_id, imu_msg->header.stamp,
                                ros::Duration(transform_tolerance_)))
      {
        transform = tfBuffer_.lookupTransform(parent_frame_, imu_msg->header.frame_id, imu_msg->header.stamp);
        tf2::doTransform(imu_msg->orientation, *imu, transform);
        imuConst = imu;
      }
      else
      {
        ROS_WARN_STREAM("Imu_in is waiting to transform cloud from " << imu_msg->header.frame_id << " to "
                                                                << parent_frame_ << ".");
        return;
      }
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
      return;
    }
  }
  else
  {
    imuConst = boost::make_shared<const geometry_msgs::Quaternion>(imu_msg->orientation);
  }

  // Extract roll, pitch, and yaw
  tf2::Quaternion quat;
  tf2::convert(*imuConst, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  quat.setRPY(-roll,-pitch, 0.0);
  
  //Publish transform from parent to child frame id's
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = imu_msg->header.stamp;
  tf_msg.header.frame_id = parent_frame_;
  tf_msg.child_frame_id  = child_frame_;
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf2::convert(quat, tf_msg.transform.rotation);
  // tf_msg.transform.rotation.x   =  quat.getX();
  // tf_msg.transform.rotation.y   =  quat.getY();
  // tf_msg.transform.rotation.z   =  quat.getZ();
  // tf_msg.transform.rotation.w   = -imuConst->orientation.w;

  br.sendTransform(tf_msg);
}
}  // namespace pointcloud_to_laserscan

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::BaseGimbalNodelet, nodelet::Nodelet)
