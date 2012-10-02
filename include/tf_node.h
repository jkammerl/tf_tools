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

#ifndef TFNODE_H_
#define TFNODE_H_

#include <set>

namespace tf_tunnel
{

  class TFTreeNode
  {
  public:
    TFTreeNode() :
      nodeID_(0),
      parent_nodeID_(0),
      parent_node_(0),
      changed_(false),
      max_delay_(0.0f),
      update_counter_(0)
    {
      first_received_ = ros::Time::now();
    }

    ~TFTreeNode()
    {
    }

    float getAvgDelay()
    {
      float avg_delay = ros::Duration(last_received_ - tf_msg_->header.stamp).toSec();
      max_delay_ = std::max<float>(max_delay_, abs(avg_delay));
      return avg_delay;
    }

    float getMaxDelay()
    {
      return max_delay_;
    }

    float getFrequency()
    {
      return (float)update_counter_ / (float)ros::Duration(ros::Time::now() - first_received_).toSec();
    }

    // node id
    unsigned int nodeID_;
    unsigned int parent_nodeID_;

    // set of child nodes
    std::set<TFTreeNode*> subnodes_;
    TFTreeNode* parent_node_;

    // most recently received tf message
    geometry_msgs::TransformStampedConstPtr tf_msg_;

    // most recently transmitted tf message
    geometry_msgs::TransformStampedConstPtr tf_msg_sent_;

    // statistics
    bool changed_;
    ros::Time last_received_;
    ros::Time first_received_;
    ros::Time transmission_stamp_;
    float max_delay_;
    unsigned int update_counter_;
  };
}

#endif
