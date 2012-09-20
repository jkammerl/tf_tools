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
 *
 *  Created on: Aug 21, 2012
 *      Author: Julius Kammerl (julius@kammerl.de)
 */

#ifndef TF_CONTAINER_H_
#define TF_CONTAINER_H_

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>

#include <boost/thread/mutex.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>

#include <map>
#include <set>
#include <ostream>
#include <stdint.h>

#include "tf_node.h"

using namespace std;

namespace tf_tools
{

/*
class FrameCounter
{
public:
  FrameCounter() : frame_count_(0) { }
  FrameCounter(uint16_t count) : frame_count_(count) { }

  virtual ~FrameCounter(){}

  FrameCounter operator++ (int)
  {
    FrameCounter result = *this;
    ++ (*this);
    return result;
  }

  FrameCounter& operator++ ()
  {
    ++(this->frame_count_);
    return *this;
  }

  uint16_t frame_count_;

private:

    friend class boost::serialization::access;
    friend std::ostream & operator<<(std::ostream &os, const FrameCounter &gp);

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & frame_count_;
    }

};
*/


class FrameHeader
{
public:
  enum frame_type_t
  {
    UNDEFINED_FRAME_TYPE, COMPRESSED_TF_CONTAINER, COMPRESSED_FRAME_ID_TABLE, FRAME_TYPE_COUNT
  };

  FrameHeader() : type_(UNDEFINED_FRAME_TYPE) { }
  FrameHeader(frame_type_t type) : type_(type) { }

  virtual ~FrameHeader(){}


  frame_type_t type_;

private:

    friend class boost::serialization::access;
    friend std::ostream & operator<<(std::ostream &os, const FrameHeader &gp);

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & type_;
    }

};


/*TF Message
 *
 geometry_msgs/TransformStamped[] transforms
 std_msgs/Header header
 uint32 seq
 time stamp
 string frame_id
 string child_frame_id
 geometry_msgs/Transform transform
 geometry_msgs/Vector3 translation
 float64 x
 float64 y
 float64 z
 geometry_msgs/Quaternion rotation
 float64 x
 float64 y
 float64 z
 float64 w
 *
 */
class TFMessageContainer
{
public:


  TFMessageContainer()
  {
  }
  virtual ~TFMessageContainer()
  {
  }

  enum update_type_t
  {
    INTRA_FRAME, PREDICTION_FRAME, UPDATE_TYPE_COUNT
  };

  void reserve(size_t size)
  {
    sequence_.reserve(size);
    timeStamp_.reserve(size);

    source_frame_id_.reserve(size);
    target_frame_id_.reserve(size);

    transform_x_.reserve(size);
    transform_y_.reserve(size);
    transform_z_.reserve(size);

    quaternion_x_.reserve(size);
    quaternion_y_.reserve(size);
    quaternion_z_.reserve(size);
    quaternion_w_.reserve(size);
  }

  void clear()
  {
    update_type_.clear();

    sequence_.clear();
    timeStamp_.clear();

    source_frame_id_.clear();
    target_frame_id_.clear();

    transform_x_.clear();
    transform_y_.clear();
    transform_z_.clear();

    quaternion_x_.clear();
    quaternion_y_.clear();
    quaternion_z_.clear();
    quaternion_w_.clear();
  }

  void add(TFTreeNode* node)
  {
    uint16_t source_frame_id, target_frame_id;

    geometry_msgs::TransformStampedConstPtr tf_msg = node->tf_msg_;
    if (tf_msg)
    {
      node->tf_msg_sent_ = tf_msg;

      source_frame_id = node->parent_nodeID_;
      target_frame_id = node->nodeID_;

      if (source_frame_id!=target_frame_id)
        add(tf_msg, source_frame_id, target_frame_id);
    }
  }

  void add(geometry_msgs::TransformStampedConstPtr tf_msg, uint16_t source_frame_id, uint16_t target_frame_id)
  {
    TFMessageContainer::update_type_t frame_type = TFMessageContainer::INTRA_FRAME;
    update_type_.push_back(frame_type);

    source_frame_id_.push_back(source_frame_id);
    target_frame_id_.push_back(target_frame_id);

    sequence_.push_back(tf_msg->header.seq);
    timeStamp_.push_back(tf_msg->header.stamp.toNSec());

    transform_x_.push_back(*reinterpret_cast<const uint64_t*>(&tf_msg->transform.translation.x));
    transform_y_.push_back(*reinterpret_cast<const uint64_t*>(&tf_msg->transform.translation.y));
    transform_z_.push_back(*reinterpret_cast<const uint64_t*>(&tf_msg->transform.translation.z));

    quaternion_x_.push_back(*reinterpret_cast<const uint64_t*>(&tf_msg->transform.rotation.x));
    quaternion_y_.push_back(*reinterpret_cast<const uint64_t*>(&tf_msg->transform.rotation.y));
    quaternion_z_.push_back(*reinterpret_cast<const uint64_t*>(&tf_msg->transform.rotation.z));
    quaternion_w_.push_back(*reinterpret_cast<const uint64_t*>(&tf_msg->transform.rotation.w));
  }

  std::size_t size()
  {
    return update_type_.size();
  }

  void decode (const size_t idx, geometry_msgs::TransformStamped& tf_msg, uint16_t& source_frame_id, uint16_t& target_frame_id)
  {
    assert (idx<=update_type_.size());

    tf_msg.header.seq = sequence_[idx];
    tf_msg.header.stamp.fromNSec(timeStamp_[idx]) ;

    source_frame_id = source_frame_id_[idx];
    target_frame_id = target_frame_id_[idx];

    tf_msg.transform.translation.x = *reinterpret_cast<const double*>(&transform_x_[idx]);
    tf_msg.transform.translation.y = *reinterpret_cast<const double*>(&transform_y_[idx]);
    tf_msg.transform.translation.z = *reinterpret_cast<const double*>(&transform_z_[idx]);

    tf_msg.transform.rotation.x = *reinterpret_cast<const double*>(&quaternion_x_[idx]);
    tf_msg.transform.rotation.y = *reinterpret_cast<const double*>(&quaternion_y_[idx]);
    tf_msg.transform.rotation.z = *reinterpret_cast<const double*>(&quaternion_z_[idx]);
    tf_msg.transform.rotation.w = *reinterpret_cast<const double*>(&quaternion_w_[idx]);

  }

private:

  friend class boost::serialization::access;
  friend std::ostream & operator<<(std::ostream &os, const TFMessageContainer &gp);

  template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
      ar & update_type_;

      // tf header
      ar & sequence_;
      ar & timeStamp_;

      // tf source & target frames
      ar & source_frame_id_;
      ar & target_frame_id_;

      // tf transformation
      ar & transform_x_;
      ar & transform_y_;
      ar & transform_z_;

      // tf rotation
      ar & quaternion_x_;
      ar & quaternion_y_;
      ar & quaternion_z_;
      ar & quaternion_w_;
    }


  // header information
  vector<uint32_t> sequence_;
  vector<uint64_t> timeStamp_;

  // frame ids
  vector<uint16_t> source_frame_id_;
  vector<uint16_t> target_frame_id_;

  // translation
  vector<uint64_t> transform_x_;
  vector<uint64_t> transform_y_;
  vector<uint64_t> transform_z_;

  // rotation
  vector<uint64_t> quaternion_x_;
  vector<uint64_t> quaternion_y_;
  vector<uint64_t> quaternion_z_;
  vector<uint64_t> quaternion_w_;

  vector<update_type_t> update_type_;

};

}

#endif /* TF_CONTAINER_H_ */
