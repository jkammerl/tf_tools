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
#include <sstream>

#include "tf_compression.h"

// include compressed TF message
#include "CompressedTF.h"

using namespace std;

namespace tf_tools
{

class TFTunnel
{
protected:

  TFCompression tf_encoding_tree_;
  TFCompression tf_decoding_tree_;

  ros::NodeHandle rootEnc_nh_;
  ros::Subscriber subEnc_;
  ros::Publisher pubEnc_;

  ros::NodeHandle rootDec_nh_;
  ros::Subscriber subDec_;
  ros::Publisher pubDec_;

  ros::NodeHandle timer_nh_;
  ros::NodeHandle priv_nh_;

  boost::mutex mutex_;
  ros::Timer sync_timer_;
  string root_node_;
  ros::CallbackQueue tf_message_encoder_callback_queue_;
  ros::CallbackQueue tf_message_decoder_callback_queue_;
  boost::thread* encoder_callback_queue_thread_;
  boost::thread* decoder_callback_queue_thread_;

  int wait_time_;
  bool verbose_;

  unsigned int tf_counter_;

  void doEncoding()
  {
    std::size_t len;
    std::stringstream compressedDataStream;

    tf_encoding_tree_.encodeCompressedTFStream(compressedDataStream);

    len = compressedDataStream.str().length();

    if (len > 0)
    {
      typedef std::istream_iterator<char> istream_iterator;

      tf_tool::CompressedTFPtr compressed_msg = tf_tool::CompressedTFPtr(new tf_tool::CompressedTF);
      compressed_msg->header.stamp = ros::Time::now();

      // copy data from stringstream to ROS message
      compressedDataStream >> std::noskipws;
      compressed_msg->data.reserve(len);
      std::copy(istream_iterator(compressedDataStream), istream_iterator(), std::back_inserter(compressed_msg->data));

      // publish message
      pubEnc_.publish(*compressed_msg);

 //     doDecoding(compressed_msg);
    }
  }

  void doDecoding(tf_tool::CompressedTFConstPtr compressed_tf_msg)
  {

    std::cout<<"decoding.."<<std::endl;
    std::stringstream compressedDataStream;

    if (compressed_tf_msg->data.size()>0)
    {
      //std::istringstream compressedDataStream(std::ios::binary);
      //compressedDataStream.rdbuf()->pubsetbuf((char*)&compressed_tf_msg->data[0], compressed_tf_msg->data.size());
      std::copy(compressed_tf_msg->data.begin(), compressed_tf_msg->data.end(),
                std::ostream_iterator < uint8_t > (compressedDataStream));

      tf::tfMessage decoded_msg;

      tf_decoding_tree_.decodeCompressedTFStream(compressedDataStream, decoded_msg);

      if (decoded_msg.transforms.size()>0)
        pubDec_.publish(decoded_msg);
    }

  }

  void callbackTF(const tf::tfMessageConstPtr &msg)
  {
    for (size_t i = 0; i < msg->transforms.size(); i++)
    {
      tf_encoding_tree_.addTFMessage(msg->transforms[i]);
      tf_counter_++;
    }
  }

public:
  TFTunnel() :
    rootEnc_nh_(""), rootDec_nh_(""), priv_nh_("~")
  {
    tf_counter_ = 0;
    string sub_topic;
    std::string pub_topic;

    // ENCODER

    // subscribe to /tf

    priv_nh_.param<string>("subscribe_topic", sub_topic, "/tf");
    subEnc_ = rootEnc_nh_.subscribe(sub_topic, 1, &TFTunnel::callbackTF, this);
    rootEnc_nh_.setCallbackQueue(&tf_message_encoder_callback_queue_);

    // publisher
    priv_nh_.param<std::string>("publish_topic", pub_topic, "/tf_compressed");
    pubEnc_ = rootEnc_nh_.advertise<tf_tool::CompressedTF>(pub_topic, 1);

    // DECODER
    priv_nh_.param<string>("subscribe_topic", sub_topic, "/tf_compressed");
    subDec_ = rootDec_nh_.subscribe(sub_topic, 1, &TFTunnel::doDecoding, this);
    rootDec_nh_.setCallbackQueue(&tf_message_decoder_callback_queue_);

    priv_nh_.param<std::string>("publish_topic", pub_topic, "/tf_tunnel");
    pubDec_ = rootDec_nh_.advertise<tf::tfMessage>(pub_topic, 1);


    // create thread for processing callback queue
    encoder_callback_queue_thread_ = new boost::thread(boost::bind(&TFTunnel::processEncoderCallbackQueueThread, this));
    decoder_callback_queue_thread_ = new boost::thread(boost::bind(&TFTunnel::processDecoderCallbackQueueThread, this));

    // read verbose parameter
    verbose_ = false;
    priv_nh_.param<bool>("verbose", verbose_, false);

    // read root node parameter
    priv_nh_.param<string>("root", root_node_, "");

    // waiting callback
    cout << "Collecting tf frames.. ";
    wait_time_ = 3;
    priv_nh_.getParam("wait", wait_time_);
    sync_timer_ = timer_nh_.createTimer(ros::Duration(1.0 / 10), boost::bind(&TFTunnel::doEncoding, this));
  }

  void processEncoderCallbackQueueThread()
  {
    tf_message_encoder_callback_queue_.callAvailable(ros::WallDuration(0.001));
  }

  void processDecoderCallbackQueueThread()
  {
    tf_message_decoder_callback_queue_.callAvailable(ros::WallDuration(0.001));
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_tunnel");
  tf_tools::TFTunnel tftree;
  ros::spin();
  return 0;
}
