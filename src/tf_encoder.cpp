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
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <sstream>

#include "tf_compression.h"

// include compressed TF message
#include "CompressedTF.h"

using namespace std;

namespace tf_tunnel
{

class TFEncoder
{
protected:

  TFCompression tf_encoding_tree_;

  ros::NodeHandle rootEnc_nh_;
  ros::Subscriber subEnc_;
  ros::Publisher pubEnc_;

  ros::NodeHandle timer_nh_;
  ros::NodeHandle priv_nh_;

  boost::mutex mutex_;
  ros::Timer sync_timer_;
  string root_node_;
  ros::CallbackQueue tf_message_encoder_callback_queue_;
  boost::thread* encoder_callback_queue_thread_;

  std::vector<std::string> frame_selector_;

  int wait_time_;
  bool verbose_;

  void doEncoding()
  {
    std::size_t len;
    std::stringstream compressedDataStream;

    compressedDataStream >> std::noskipws;

    tf_encoding_tree_.encodeCompressedTFStream(compressedDataStream, frame_selector_);

    len = compressedDataStream.str().length();

    if (len > 0)
    {
      ROS_DEBUG("Compressed data size: %d bytes", (int)len);

      // Create ROS message
      typedef std::istream_iterator<char> istream_iterator;

      tf_tunnel::CompressedTFPtr compressed_msg = tf_tunnel::CompressedTFPtr(new tf_tunnel::CompressedTF);
      compressed_msg->header.stamp = ros::Time::now();

      // copy data from stringstream to ROS message
      compressed_msg->data.reserve(len);
      std::copy(istream_iterator(compressedDataStream), istream_iterator(), std::back_inserter(compressed_msg->data));

      // publish message
      pubEnc_.publish(*compressed_msg);
    }
  }


  void callbackTF(const tf::tfMessageConstPtr &msg)
  {
    tf_encoding_tree_.addTFMessage(*msg);
  }

  // Handles (un)subscribing when clients (un)subscribe
  void connectCb()
  {
    tf_encoding_tree_.triggerIntraUpdate();
  }

public:
  TFEncoder() :
    rootEnc_nh_(""), priv_nh_("~")
  {

    // ENCODER

    // subscribe to /tf
    string sub_topic;
    priv_nh_.param<string>("subscribe_topic", sub_topic, "/tf");
    subEnc_ = rootEnc_nh_.subscribe(sub_topic, 1, &TFEncoder::callbackTF, this);
    rootEnc_nh_.setCallbackQueue(&tf_message_encoder_callback_queue_);

    // publisher
    ros::SubscriberStatusCallback connect_cb = boost::bind(&TFEncoder::connectCb, this);

    string pub_topic;
    priv_nh_.param<std::string>("publish_topic", pub_topic, "/tf_compressed");
    pubEnc_ = rootEnc_nh_.advertise<tf_tunnel::CompressedTF>(pub_topic, 1, connect_cb);

    // create thread for processing callback queue
    encoder_callback_queue_thread_ = new boost::thread(boost::bind(&TFEncoder::processEncoderCallbackQueueThread, this));

    // read verbose parameter
    verbose_ = false;
    priv_nh_.param<bool>("verbose", verbose_, false);

    string frame_selector;
    priv_nh_.param<std::string>("frame_selector", frame_selector, "");
    initializeFrameFilter(frame_selector);

    double rate;
    priv_nh_.param<double>("rate", rate, 10.0);
    sync_timer_ = timer_nh_.createTimer(ros::Duration(1.0 / rate), boost::bind(&TFEncoder::doEncoding, this));
  }

protected:
  void initializeFrameFilter(const string& selector_str_arg)
  {
    std::vector<std::string> temp_selection_strings;
    boost::split(temp_selection_strings, selector_str_arg, boost::is_any_of(","));

    vector<string>::iterator it;
    vector<string>::iterator it_end=temp_selection_strings.end();

    frame_selector_.clear();
    frame_selector_.reserve(temp_selection_strings.size());

    for (it=temp_selection_strings.begin(); it!=it_end; ++it)
    {
      boost::algorithm::trim(*it);

      if (!(*it).empty())
      {
        if ((*it)[0]!='/')
          *it="/"+*it;

        frame_selector_.push_back(*it);

        ROS_INFO("Selected frame: %s", frame_selector_.back().c_str());
      }
    }

  }

  void processEncoderCallbackQueueThread()
  {
    tf_message_encoder_callback_queue_.callAvailable(ros::WallDuration(0.001));
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_encoder");
  tf_tunnel::TFEncoder tftree;
  ros::spin();
  return 0;
}
