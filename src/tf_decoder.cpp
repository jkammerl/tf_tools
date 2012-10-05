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

namespace tf_tunnel
{

class TFDecoder
{
protected:

  TFCompression tf_decoding_tree_;


  ros::NodeHandle rootDec_nh_;
  ros::Subscriber subDec_;
  ros::Publisher pubDec_;

  ros::NodeHandle timer_nh_;
  ros::NodeHandle priv_nh_;

  boost::mutex mutex_;
  ros::Timer sync_timer_;
  string root_node_;

  ros::CallbackQueue tf_message_decoder_callback_queue_;
  boost::thread* decoder_callback_queue_thread_;

  int wait_time_;
  bool verbose_;

  unsigned int tf_counter_;

  void doDecoding(tf_tunnel::CompressedTFConstPtr compressed_tf_msg)
  {

    std::cout<<"decoding.."<<std::endl;
    std::stringstream compressedDataStream;

    if (compressed_tf_msg->data.size()>0)
    {
      //std::istringstream compressedDataStream(std::ios::binary);
      //compressedDataStream.rdbuf()->pubsetbuf((char*)&compressed_tf_msg->data[0], compressed_tf_msg->data.size());
      std::copy(compressed_tf_msg->data.begin(), compressed_tf_msg->data.end(),
                std::ostream_iterator < uint8_t > (compressedDataStream));

      tf_decoding_tree_.decodeCompressedTFStream(compressedDataStream);

    }

  }

  void outputTF()
  {


    tf_decoding_tree_.showTFTree();

    tf_decoding_tree_.showTFRoots();


    tf::tfMessage decoded_msg;

    tf_decoding_tree_.getTFMessage(decoded_msg);

    if (decoded_msg.transforms.size()>0)
      pubDec_.publish(decoded_msg);

  }

public:
  TFDecoder() :
    rootDec_nh_(""), priv_nh_("~")
  {
    tf_counter_ = 0;
    string sub_topic;
    std::string pub_topic;

    // DECODER
    priv_nh_.param<string>("subscribe_topic", sub_topic, "/tf_compressed");
    subDec_ = rootDec_nh_.subscribe(sub_topic, 1, &TFDecoder::doDecoding, this);
    rootDec_nh_.setCallbackQueue(&tf_message_decoder_callback_queue_);

    priv_nh_.param<std::string>("publish_topic", pub_topic, "/tf_decoded");
    pubDec_ = rootDec_nh_.advertise<tf::tfMessage>(pub_topic, 1);

    // create thread for processing callback queue
    decoder_callback_queue_thread_ = new boost::thread(boost::bind(&TFDecoder::processDecoderCallbackQueueThread, this));

    // read verbose parameter
    verbose_ = false;
    priv_nh_.param<bool>("verbose", verbose_, false);

    double rate;
    priv_nh_.param<double>("rate", rate, 10.0);
    sync_timer_ = timer_nh_.createTimer(ros::Duration(1.0 / rate), boost::bind(&TFDecoder::outputTF, this));


  }

  void processDecoderCallbackQueueThread()
  {
    tf_message_decoder_callback_queue_.callAvailable(ros::WallDuration(0.001));
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_decoder");
  tf_tunnel::TFDecoder tftree;
  ros::spin();
  return 0;
}
