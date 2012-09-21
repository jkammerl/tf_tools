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
#include "boost/thread.hpp"

#include <ros/ros.h>
#include "ros/callback_queue.h"

#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>


#include "tf_tree.h"

using namespace std;

namespace tf_tunnel
{

class ShowTFTree
{
protected:

  TFTree tf_tree_;
  ros::NodeHandle root_nh_;
  ros::NodeHandle timer_nh_;
  ros::NodeHandle priv_nh_;
  ros::Subscriber sub_;
  boost::mutex mutex_;
  ros::Timer sync_timer_;
  string root_node_;
  ros::CallbackQueue tf_message_callback_queue_;

  int wait_time_;
  bool verbose_;

  unsigned int tf_counter_;

  void wait()
   {
    static int call_counter = 0;

    if (wait_time_-call_counter==0)
    {
      cout<<"0"<<endl;
      tf_tree_.showTFRoots();
      ros::shutdown();
    } else
    {
      cout << wait_time_-call_counter<<"..";
      cout.flush();
    }

    ++call_counter;
   }

  void callback(const tf::tfMessageConstPtr &msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    for (size_t i = 0; i < msg->transforms.size(); i++)
    {
      tf_tree_.addTFMessage(msg->transforms[i]);
      tf_counter_++;
    }
  }

public:
  ShowTFTree() : root_nh_(""), priv_nh_("~")
  {
    tf_counter_ = 0;

    // subscribe to /tf
    string sub_topic;
    priv_nh_.param<string>("subscribe_topic", sub_topic, "/tf");

    sub_ = root_nh_.subscribe(sub_topic, 10, &ShowTFTree::callback, this);
    root_nh_.setCallbackQueue(&tf_message_callback_queue_);

    // create thread for processing callback queue
    boost::thread* dedicated_listener_thread_;
    dedicated_listener_thread_ = new boost::thread(boost::bind(&ShowTFTree::processCallbackQueueThread, this));

    // waiting callback
    cout <<"Collecting tf frames.. " ;
    wait_time_ = 3;
    priv_nh_.getParam("wait", wait_time_);
    sync_timer_ =  timer_nh_.createTimer(ros::Duration(1.0), boost::bind( &ShowTFTree::wait, this ) );
  }

  void processCallbackQueueThread()
  {
      tf_message_callback_queue_.callAvailable(ros::WallDuration(0.01));
  };

};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_tf_roots");
  tf_tunnel::ShowTFTree tftree;
  ros::spin();
  return 0;
}
