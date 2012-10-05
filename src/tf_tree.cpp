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

using namespace std;

namespace tf_tunnel
{


  void TFTree::addTFMessage (const tf::tfMessage& msg)
  {
    boost::mutex::scoped_lock lock (mutex_);

    size_t i;

    for (i = 0; i < msg.transforms.size(); i++)
    {
      const geometry_msgs::TransformStamped trans_msg = msg.transforms[i];


      addTFMessage(trans_msg);
    }
  }

  void TFTree::addTFMessage (const geometry_msgs::TransformStamped& trans_msg)
  {


    if (most_recent_tf_time_stamp_<trans_msg.header.stamp)
    {
      most_recent_tf_time_stamp_ = trans_msg.header.stamp;
    }


    const string base_key = trans_msg.header.frame_id;
    const string target_key = trans_msg.child_frame_id;

    map<string, unsigned int>::iterator it_id;
    map<unsigned int, TFTreeNode*>::iterator it_node;

    /// SOURCE FRAME

    TFTreeNode* tf_source_node = 0;
    unsigned int tf_source_id;

    it_id = frameStr_to_frameID_lookup_.find (base_key);
    if (it_id != frameStr_to_frameID_lookup_.end ())
    {
      // node exists
      tf_source_id = it_id->second;

      // lookup node pointer
      assert (tf_source_id<frameID_to_nodePtr_lookup_.size());
      tf_source_node = frameID_to_nodePtr_lookup_[tf_source_id];
    }
    else
    {
      // node does not exist

      // assign frame id
      tf_source_id = frame_count_++;

      // add entries to lookup maps
      frameStr_to_frameID_lookup_[base_key] = tf_source_id;
      frameID_to_frameStr_lookup_.push_back(base_key);

      // create new node and add it to frameID_to_nodePtr_lookup_ map
      tf_source_node = new TFTreeNode ();
      tf_source_node->nodeID_ = tf_source_id;

      frameID_to_nodePtr_lookup_.push_back(tf_source_node);

      // new nodes are root nodes until being linked
      root_nodes_.insert(tf_source_node);

      dirty_frameID_table_ = true;

    }

    /// TARGET FRAME

    TFTreeNode* tf_target_node = 0;
    unsigned int tf_target_id;

    it_id = frameStr_to_frameID_lookup_.find (target_key);
    if (it_id != frameStr_to_frameID_lookup_.end ())
    {
      // node exists
      tf_target_id = it_id->second;

      // lookup node pointer
      assert (tf_target_id<frameID_to_nodePtr_lookup_.size());
      tf_target_node = frameID_to_nodePtr_lookup_[tf_target_id];
    }
    else
    {
      // node does not exist

      // assign frame id
      tf_target_id = frame_count_++;

      // add entries to lookup maps
      frameStr_to_frameID_lookup_[target_key] = tf_target_id;
      frameID_to_frameStr_lookup_.push_back(target_key);

      // create new node and add it to frameID_to_nodePtr_lookup_ map
      tf_target_node = new TFTreeNode ();

      frameID_to_nodePtr_lookup_.push_back(tf_target_node);

      // new nodes are root nodes until being linked
      root_nodes_.insert(tf_target_node);

      dirty_frameID_table_ = true;
    }

    assert (tf_source_node && tf_target_node);

    // updating node

    tf_target_node->nodeID_ = tf_target_id;

    // link nodes
    tf_source_node->subnodes_.insert (tf_target_node);
    tf_target_node->parentPtr_ = tf_source_node;

    root_nodes_.erase(tf_target_node);

    if (!tf_source_node->hasParent())
    {
      root_nodes_.insert(tf_source_node);
    }

    tf_target_node->tf_msg_ = geometry_msgs::TransformStampedPtr(new geometry_msgs::TransformStamped(trans_msg));
    tf_target_node->changed_ = true;
    tf_target_node->last_received_ = ros::Time::now ();
    tf_target_node->update_counter_++;

  }


  void TFTree::getTFMessage(tf::tfMessage& msg)
  {
    ros::Time time_latest = ros::Time::now();

    msg.transforms.reserve(frameID_to_nodePtr_lookup_.size());

    if (time_latest<most_recent_tf_time_stamp_)
      time_latest = most_recent_tf_time_stamp_;

    vector<TFTreeNode*>::iterator it;
    vector<TFTreeNode*>::const_iterator it_end = frameID_to_nodePtr_lookup_.end();
    for (it = frameID_to_nodePtr_lookup_.begin(); it != it_end; ++it)
    {
      geometry_msgs::TransformStampedConstPtr stamped_msg = (*it)->tf_msg_;
      if (stamped_msg)
      {
        msg.transforms.push_back(*stamped_msg);

        geometry_msgs::TransformStamped& back = msg.transforms.back();

        back.header.stamp = time_latest;

        string& source_frame = back.header.frame_id;
        string& target_frame = back.child_frame_id;

        if (!source_frame.empty())
          source_frame = "/"+prefix_+source_frame.substr(1);
        if (!target_frame.empty())
          target_frame = "/"+prefix_+target_frame.substr(1);
      }
    }
  }


  void TFTree::showTFRoots ()
  {

    // vector of root node (IDs)
    vector<unsigned int> tf_root_nodes;

    cout << "Number of roots in TF stream: " << (int)root_nodes_.size () << endl;

    std::set<TFTreeNode*>::iterator it;
    std::set<TFTreeNode*>::const_iterator it_end = root_nodes_.end();

    for (it = root_nodes_.begin(); it != it_end; ++it)
    {
      cout <<" - "<< frameID_to_frameStr_lookup_[(*it)->nodeID_]<<endl;
    }
  }

  void TFTree::showTFTree(bool verbose)
  {
    std::set<TFTreeNode*>::iterator it;
    std::set<TFTreeNode*>::const_iterator it_end = root_nodes_.end();

    for (it = root_nodes_.begin(); it != it_end; ++it)
    {
      showTFTree (*it, verbose);
    }

    return;
  }

  void TFTree::showTFTree (const string& root, bool verbose)
  {
    string root_node = root;
    map<string, unsigned int>::iterator it;

    if (root.empty())
    {
      // no root node specified
      showTFTree(verbose);
      return;
    }

    // search for root node string
    it = frameStr_to_frameID_lookup_.find (root_node);
    if (it != frameStr_to_frameID_lookup_.end ())
    {
      showTFTree (it->second, verbose);
      return;
    }

    // search for root node string - remove trailing "/"
    it = frameStr_to_frameID_lookup_.find ("/" + root_node);
    if (it != frameStr_to_frameID_lookup_.end ())
    {
      showTFTree (it->second, verbose);
      return;
    }

    ROS_ERROR("Root not found: %s", root.c_str());

  }

  void TFTree::showTFTree(unsigned int rootID, bool verbose)
  {
    assert (rootID<frameID_to_nodePtr_lookup_.size());
    showTFTree(frameID_to_nodePtr_lookup_[rootID], verbose);
  }

  void TFTree::showTFTree (TFTreeNode* root, bool verbose)
  {
    boost::mutex::scoped_lock lock (mutex_);

    string node_name;
    map<unsigned int, TFTreeNode*>::iterator it;

    TFTreeIterator tf_it(root);

    vector<bool> tree_connector;
    tree_connector.reserve (frame_count_);

    if (*tf_it && (tf_it.getBaseTFFrame ()))
    {
      // get node string
      node_name = frameID_to_frameStr_lookup_[tf_it.getBaseTFFrame ()->nodeID_];
      node_name.erase (0, 1);

      cout << "+--" << node_name << endl;
      tree_connector.push_back (true);
    }

    while (*tf_it)
    {
      unsigned char depth = tf_it.getDepth ();
      bool last_child = tf_it.isLastChild ();

      // mark remaining childs for every tree level
      if (depth >= tree_connector.size ())
      {
        tree_connector.push_back (last_child);
      }
      else
      {
        tree_connector[depth] = last_child;
      }

      // get node string
      node_name = frameID_to_frameStr_lookup_[tf_it.getTargetTFFrame ()->nodeID_];

      // remove trailing "/"
      node_name.erase (0, 1);

      // draw ascii connectors
      for (unsigned char i = 0; i < depth; ++i)
        if (tree_connector[i])
        {
          cout << "   ";
        }
        else
        {
          cout << "|  ";
        }
      cout << "+--" << node_name;

      if (verbose)
      {
        // print verbose information

        printf (" [Average Delay: %.4fs, Max Delay: %.4fs, Freq: %.4fHz]",
            tf_it->getAvgDelay (), tf_it->getMaxDelay (),
            tf_it->getFrequency ());
      }
      cout << endl;

      // increase tree iterator
      ++tf_it;
    }

  }

  void TFTree::removeNode(TFTreeNode* node)
  {
    // release tf msg data
    node->tf_msg_.reset();
    node->tf_msg_sent_.reset();

    // update children
    set<TFTreeNode*>::iterator it;
    set<TFTreeNode*>::const_iterator it_end = node->subnodes_.end();
    for (it = node->subnodes_.begin(); it!=it_end; ++it)
    {
      TFTreeNode* child = *it;
      child->parentPtr_ = 0;
      root_nodes_.insert(child);
    }
    node->subnodes_.clear();
    root_nodes_.erase(node);

    // update parent
    if (node->hasParent())
    {
      TFTreeNode* parent = node->parentPtr_;
      parent->subnodes_.erase(node);
      if (!parent->hasChildren())
      {
         removeNode(node->parentPtr_);
      }
      node->parentPtr_ = 0;
    }


  }


}
