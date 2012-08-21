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

#include "tf_tree.h"

namespace tf_tools
{

void TFTree::addTFMessage(const geometry_msgs::TransformStamped& msg)
{

  map<string, TFTreeNode*>::iterator it;

  const string& base_key = msg.header.frame_id;
  const string& target_key = msg.child_frame_id;

  it = tf_tree_nodes_.find(target_key);
  TFTreeNode* tf_sub_node;
  if (it != tf_tree_nodes_.end())
  {
    tf_sub_node = it->second;
  }
  else
  {
    tf_sub_node = new TFTreeNode();
    tf_tree_nodes_[target_key] = tf_sub_node;
  }

  it = tf_tree_nodes_.find(base_key);
  TFTreeNode* tf_base_node;
  if (it != tf_tree_nodes_.end())
  {
    tf_base_node = it->second;
  }
  else
  {
    tf_base_node = new TFTreeNode();
    tf_tree_nodes_[base_key] = tf_base_node;
  }

  tf_base_node->subnodes_[target_key] = tf_sub_node;
  tf_sub_node->tf_msg_ = msg;
  tf_sub_node->changed_ = true;
  tf_sub_node->last_received_ = ros::Time::now();
  tf_sub_node->update_counter_++;

  //tf_base_node->update(msg->transforms[i]);

  tf_full_nodes_set_.insert(base_key);
  tf_nodes_with_parents_.insert(target_key);
}

void TFTree::searchForRootNodes()
{
  tf_root_nodes_.clear();
  tf_root_nodes_.reserve(max<size_t>(tf_full_nodes_set_.size(), tf_nodes_with_parents_.size()));

  set_difference(tf_full_nodes_set_.begin(), tf_full_nodes_set_.end(),
                 tf_nodes_with_parents_.begin(), tf_nodes_with_parents_.end(),
                 insert_iterator<vector<string> >(tf_root_nodes_, tf_root_nodes_.end()));
}

void TFTree::showTFTree(const string& root, bool verbose)
{

  map<string, TFTreeNode*>::iterator it;

  it = tf_tree_nodes_.find(root);
  if (it != tf_tree_nodes_.end())
  {
    TFTreeIterator tf_it(it->second);

    while (*tf_it)
    {
      unsigned char depth = tf_it.getDepth();

      string node_name;

      if (depth == 1)
      {
        node_name = tf_it.getBaseTFFrame();
      }
      else
      {
        node_name = tf_it.getTargetTFFrame();
      }

      for (unsigned char i = 0; i < depth-1; ++i)
        cout << "|  ";

      cout << "+" << node_name;

      if (verbose)
      {
        printf(" [Average Delay: %.4fs, Max Delay: %.4fs, Freq: %.4fHz]", tf_it->getAvgDelay()
                                                                        , tf_it->getMaxDelay()
                                                                        , tf_it->getFrequency());
      }
      cout<< endl;

      tf_it++;
    }

  }
}

}
