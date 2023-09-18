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

#ifndef POINTCLOUD_TO_LASERSCAN_BASE_GIMBAL_NODELET_H
#define POINTCLOUD_TO_LASERSCAN_BASE_GIMBAL_NODELET_H

#include <boost/thread/mutex.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pointcloud_to_laserscan
{

//! \brief The BaseGimbalNodelet class to provide an always level frame of reference.
//!
class BaseGimbalNodelet : public nodelet::Nodelet
{
public:
  BaseGimbalNodelet();

private:
  virtual void onInit();

  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber sub_;  

  tf2_ros::TransformListener tfListener_;
  tf2_ros::Buffer tfBuffer_;

  // ROS Parameters
  std::string child_frame_;
  std::string parent_frame_;
  double transform_tolerance_;
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN_BASE_GIMBAL_NODELET_H
