/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>

#include "tf_tree.h"

using namespace std;

namespace tf_tools
{

class TF_Tool
{
protected:

  TFTree tf_tree_;
  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;
//  ros::Publisher pub_;
  ros::Subscriber sub_;
  boost::mutex mutex_;
  ros::Timer sync_timer_;
  //ros::Duration republish_time_;
  //bool use_diff_;
  //double linear_change_threshold_;
  //double angular_change_threshold_;




  void sync()
   {
     boost::mutex::scoped_lock lock(mutex_);

     tf_tree_.showTFTree();

     tf_tree_.showTFPath("/head_pan_link", "/wide_stereo_l_stereo_camera_frame");
   }

  void callback(const tf::tfMessageConstPtr &msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    for (size_t i = 0; i < msg->transforms.size(); i++)
    {
      tf_tree_.addTFMessage(msg->transforms[i]);
    }
  }

public:
  TF_Tool() : root_nh_(""), priv_nh_("~")
  {
    string pub_topic;
//    priv_nh_.param<string>("publish_topic", pub_topic, "/tf_throttled");
 //   pub_ = root_nh_.advertise<tf::tfMessage>(pub_topic, 10);
    string sub_topic;
    priv_nh_.param<string>("subscribe_topic", sub_topic, "/tf");
    sub_ = root_nh_.subscribe(sub_topic, 10, &TF_Tool::callback, this);
    double rate;
    priv_nh_.param<double>("rate", rate, 10.0);
    sync_timer_ =  root_nh_.createTimer(ros::Duration(1.0/rate), boost::bind( &TF_Tool::sync, this ) );
/*    double repub_time;
    priv_nh_.param<double>("republish_time", repub_time, 0.5);
    republish_time_ = ros::Duration(repub_time);    
    priv_nh_.param<bool>("use_diff", use_diff_, true);
    priv_nh_.param<double>("angular_change_threshold", angular_change_threshold_, 0.02);
    priv_nh_.param<double>("linear_change_threshold", linear_change_threshold_, 1.0);
    ROS_INFO("TF throttle started; listening to %s and publishing on %s at %f Hz", sub_topic.c_str(),
	     pub_topic.c_str(), rate);
	     */
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_throttle");
  tf_tools::TF_Tool throttle;
  ros::spin();
  return 0;
}
