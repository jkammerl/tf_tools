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

#ifndef TFTREE_H_
#define TFTREE_H_

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
#include <vector>
#include <set>
#include <ostream>
#include <stdint.h>

#include "tf_node.h"
#include "tf_tree_iterator.h"

namespace tf_tunnel
{

class FrameVector : public std::vector<std::string>
{
  friend class boost::serialization::access;
  friend std::ostream & operator<<(std::ostream &os, const FrameVector &gp);

public:
  FrameVector() { }
  virtual ~FrameVector() { }

private:
  template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
      std::vector < std::string > &vec = *this;
      ar & vec;
    }
};

class TFTree
{
public:
  TFTree() :
      frame_count_(0), dirty_frameID_table_(false) { }
  virtual ~TFTree() { }

  void addTFMessage(const tf::tfMessage& msg);
  void addTFMessage(const geometry_msgs::TransformStamped& trans_msg);
  void getTFMessage(tf::tfMessage& msg);

  void showTFRoots();

  void showTFTree(bool verbose = true);
  void showTFTree(unsigned int rootID, bool verbose = true);
  void showTFTree(TFTreeNode* root, bool verbose = true);
  void showTFTree(const std::string& root, bool verbose = true);

  void setDecodingPrefix(std::string& prefix)
  {
    prefix_ = prefix;
  }

protected:
  void removeNode(TFTreeNode* node);

  boost::mutex mutex_;

  // Lookup tables and maps
  std::map<std::string, unsigned int> frameStr_to_frameID_lookup_;
  FrameVector frameID_to_frameStr_lookup_;

  std::vector<TFTreeNode*> frameID_to_nodePtr_lookup_;

  std::set<TFTreeNode*> root_nodes_;

  // amount of frames/nodes
  unsigned int frame_count_;


  // dirty flag for frame ID table
  bool dirty_frameID_table_;

  ros::Time most_recent_tf_time_stamp_;

  ros::Duration expire_time_;

  std::string prefix_;

};

}

#endif /* TFTREE_H_ */
