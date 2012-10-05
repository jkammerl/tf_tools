/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Aug 21, 2012
 *      Author: Julius Kammerl (julius@kammerl.de)
 */
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/flush.hpp>

#include <iostream>

#include "tf_compression.h"

#include <zlib.h> // for MAX_WBITS

using namespace std;

namespace tf_tunnel
{

bool TFCompression::hasTFNodeChanged(TFTreeNode* node)
{
  if (!node->tf_msg_)
    return false;
  if (!node->tf_msg_sent_)
    return true;
  if (!node->changed_)
    return false;

  tf::Transform tf1, tf2;
  tf::transformMsgToTF(node->tf_msg_->transform, tf1);
  tf::transformMsgToTF(node->tf_msg_sent_->transform, tf2);

  if (linear_change_threshold_ == 0.0 || angular_change_threshold_ == 0.0
      || tf1.getOrigin().distance(tf2.getOrigin()) > linear_change_threshold_
      || tf1.getRotation().angle(tf2.getRotation()) > angular_change_threshold_)
    return true;
  return false;

}

bool TFCompression::intraUpdateRequired(TFTreeNode* node)
{

  if (!node->tf_msg_)
    return false;
  if (!node->tf_msg_sent_)
    return true;

  return (ros::Duration(ros::Time::now() - node->transmission_stamp_).toSec() > intra_update_rate_);

}

bool TFCompression::TFTimeOut(TFTreeNode* node)
{
  if (!node->tf_msg_)
    return false;

  return (ros::Duration(ros::Time::now() - node->tf_msg_->header.stamp).toSec() > time_out_interval_);
}

void TFCompression::encodeCompressedTFStream(ostream& compressedDataOut_arg,
                                             const vector<string>& frame_selector_arg)
{
  stringstream uncompressedData;
  boost::archive::binary_oarchive oa(uncompressedData, boost::archive::no_header);

  if (ros::Duration(ros::Time::now() - last_frame_table_transmission_).toSec() > intra_update_rate_)
    dirty_frameID_table_ = true;

  if (dirty_frameID_table_)
  {

    dirty_frameID_table_ = false;
    last_frame_table_transmission_ = ros::Time::now();

    FrameHeader header(FrameHeader::COMPRESSED_FRAME_ID_TABLE);

    oa << header;
    {
      boost::mutex::scoped_lock lock(mutex_);
      oa << frameID_to_frameStr_lookup_;

      ROS_DEBUG("Encoding FrameID table (size: %d)", (int)frameID_to_frameStr_lookup_.size());
    }
  }

  // encode all nodes

  {
    TFContainer_.clear();
    TFContainer_.reserve(frameID_to_nodePtr_lookup_.size());

    TFTimeOut_.clear();
    TFTimeOut_.reserve(frameID_to_nodePtr_lookup_.size());

    if (!frame_selector_arg.size())
    {
      // encode all tf message
      boost::mutex::scoped_lock lock(mutex_);

      //ROS_DEBUG("Encoding collected TF messages (size: %d)", (int)frameID_to_nodePtr_lookup_.size());

      vector<TFTreeNode*>::iterator it;
      ;
      vector<TFTreeNode*>::iterator it_end = frameID_to_nodePtr_lookup_.end();
      for (it = frameID_to_nodePtr_lookup_.begin(); it != it_end; ++it)
      {
        if (hasTFNodeChanged(*it) || intraUpdateRequired(*it))
        {
          TFContainer_.add(*it);
        }

        if (TFTimeOut(*it))
        {
          TFTimeOut_.push_back((*it)->nodeID_);
          removeNode(*it);
        }
      }
    }
    else
    {
      // encode only selected tf message
      boost::mutex::scoped_lock lock(mutex_);

      vector<string>::const_iterator it_selector;
      vector<string>::const_iterator it_selector_end = frame_selector_arg.end();

      for (it_selector = frame_selector_arg.begin(); it_selector != it_selector_end; ++it_selector)
      {
        map<string, unsigned int>::iterator it_node_id;

        it_node_id = frameStr_to_frameID_lookup_.find(*it_selector);
        if (it_node_id != frameStr_to_frameID_lookup_.end())
        {
          // node exists
          TFTreeNode* node = frameID_to_nodePtr_lookup_[it_node_id->second];
          do
          {
            if (hasTFNodeChanged(node) || intraUpdateRequired(node))
            {
              TFContainer_.add(node);
            }

            if (TFTimeOut(node))
            {
              TFTimeOut_.push_back(node->nodeID_);
              removeNode(node);
            }

            node = node->parentPtr_;
          } while (node);
        }
      }

    }

    if (TFContainer_.size() > 0)
    {
      FrameHeader header(FrameHeader::COMPRESSED_TF_CONTAINER);
      oa << header;
      oa << TFContainer_;
      ROS_DEBUG("Amount of TF messages in TF container: %d", (int)TFContainer_.size());
    }

    if (TFTimeOut_.size() > 0)
    {
      FrameHeader header(FrameHeader::TF_TIMEOUT_CONTAINER);
      oa << header;
      oa << TFTimeOut_;
      ROS_DEBUG("Amount of timed-out TF messages: %d", (int)TFTimeOut_.size());
    }
  }
  // zlib compression

  uncompressedData.flush();

  if (uncompressedData.str().length() > 0)
  {
    ROS_DEBUG("Uncompressed data size: %d bytes", (int)uncompressedData.str().length());

    try
    {
      boost::iostreams::zlib_params p;
      p.window_bits = 16 + MAX_WBITS;

      boost::iostreams::filtering_streambuf < boost::iostreams::output > out;
      out.push(boost::iostreams::zlib_compressor(p));
      out.push(compressedDataOut_arg);
      boost::iostreams::copy(uncompressedData, out);
      compressedDataOut_arg.flush();

    }
    catch (boost::iostreams::zlib_error& e)
    {
      ROS_ERROR("ZLIB encoding failed: %s (%d)", e.what(), e.error());
    }
  }
}

void TFCompression::decodeCompressedTFStream(istream& compressedDataIn_arg)
{

  stringstream uncompressedData;

  // zlib decompression
  try
  {
    boost::iostreams::zlib_params p;
    p.window_bits = 16 + MAX_WBITS;

    boost::iostreams::filtering_streambuf < boost::iostreams::input > in;
    in.push(boost::iostreams::zlib_decompressor(p));
    in.push(compressedDataIn_arg);
    boost::iostreams::copy(in, uncompressedData);

  }
  catch (boost::iostreams::zlib_error& e)
  {
    ROS_ERROR("ZLIB decoding failed: %s (%d)", e.what(), e.error());
  }

  // deserialize data

  boost::archive::binary_iarchive ia(uncompressedData, boost::archive::no_header);

  FrameHeader header;
  try
  {
    boost::mutex::scoped_lock lock(mutex_);

    while (1)
    {
      ia >> header;

      if (most_recent_tf_time_stamp_<ros::Time(0))
        most_recent_tf_time_stamp_ = ros::Time(0);

      switch (header.type_)
      {
        case FrameHeader::COMPRESSED_FRAME_ID_TABLE:
          size_t i;

          ia >> frameID_to_frameStr_lookup_;

          frameStr_to_frameID_lookup_.clear();
          for (i=0; i<frameID_to_frameStr_lookup_.size(); ++i)
            frameStr_to_frameID_lookup_[frameID_to_frameStr_lookup_[i]] = i;

//          frameID_to_nodePtr_lookup_.resize(frameID_to_frameStr_lookup_.size(), 0);
          frameID_to_nodePtr_lookup_.reserve(frameStr_to_frameID_lookup_.size());
          for (i=frameID_to_nodePtr_lookup_.size(); i<frameStr_to_frameID_lookup_.size(); ++i)
            frameID_to_nodePtr_lookup_.push_back(new TFTreeNode());

          ROS_DEBUG("FrameVector received (size: %d)", (int)frameID_to_frameStr_lookup_.size());
          break;
        case FrameHeader::COMPRESSED_TF_CONTAINER:
        {
          geometry_msgs::TransformStamped tf;

          uint16_t source_frame_id, target_frame_id;
          size_t table_size = frameID_to_frameStr_lookup_.size();

          TFContainer_.clear();
          ia >> TFContainer_;
          ROS_DEBUG("Compressed TF container received (size: %d)", (int)TFContainer_.size());

          for (i = 0; i < TFContainer_.size(); ++i)
          {
            TFContainer_.decode(i, tf, source_frame_id, target_frame_id);

            if ((source_frame_id < table_size) && (target_frame_id < table_size))
            {
              tf.header.frame_id = frameID_to_frameStr_lookup_[source_frame_id];
              tf.child_frame_id = frameID_to_frameStr_lookup_[target_frame_id];

              addTFMessage(tf);

              cout << tf.header.frame_id << "--" << tf.child_frame_id << endl;
            }
          }
          break;
        }
        case FrameHeader::TF_TIMEOUT_CONTAINER:
        {
          geometry_msgs::TransformStampedPtr tf;

          size_t table_size = frameID_to_frameStr_lookup_.size();

          TFTimeOut_.clear();
          ia >> TFTimeOut_;
          ROS_DEBUG("Compressed TF container received (size: %d)", (int)TFContainer_.size());

          for (i = 0; i < TFTimeOut_.size(); ++i)
          {
            if (TFTimeOut_[i] < table_size)
            {
              removeNode(frameID_to_nodePtr_lookup_[TFTimeOut_[i]]);
            }
          }
          break;
        }

        default:
          ROS_DEBUG("Received unknown container");
          break;
      }
    }
  }
  catch (boost::archive::archive_exception& e)
  {
  }

}

}
