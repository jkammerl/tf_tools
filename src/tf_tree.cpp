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

  void TFTree::addTFMessage (const geometry_msgs::TransformStamped& msg)
  {
    boost::mutex::scoped_lock lock (mutex_);

    const string& base_key = msg.header.frame_id;
    const string& target_key = msg.child_frame_id;

    map<string, unsigned int>::iterator it_id;
    map<unsigned int, TFTreeNode*>::iterator it_node;

    /// SOURCE FRAME

    TFTreeNode* tf_source_node = 0;
    unsigned int tf_source_id;

    it_id = frame_to_frameID_lookup_.find (base_key);
    if (it_id != frame_to_frameID_lookup_.end ())
    {
      // node exists
      tf_source_id = it_id->second;

      // lookup node pointer
      if (tf_source_id<frameID_to_node_lookup_.size())
        tf_source_node = frameID_to_node_lookup_[tf_source_id];
    }
    else
    {
      // node does not exist

      // assign frame id
      tf_source_id = frame_count_++;

      // add entries to lookup maps
      frame_to_frameID_lookup_[base_key] = tf_source_id;
      frameID_to_frame_lookup_.vec.push_back(base_key);

      // create new node and add it to frameID_to_node_lookup_ map
      tf_source_node = new TFTreeNode ();
      tf_source_node->nodeID_ = tf_source_id;
      frameID_to_node_lookup_.push_back(tf_source_node);

      dirty_frameID_table_ = true;

    }

    /// TARGET FRAME

    TFTreeNode* tf_target_node = 0;
    unsigned int tf_target_id;

    it_id = frame_to_frameID_lookup_.find (target_key);
    if (it_id != frame_to_frameID_lookup_.end ())
    {
      // node exists
      tf_target_id = it_id->second;

      // lookup node pointer
      if (tf_target_id<frameID_to_node_lookup_.size())
        tf_target_node = frameID_to_node_lookup_[tf_target_id];
    }
    else
    {
      // node does not exist

      // assign frame id
      tf_target_id = frame_count_++;

      // add entries to lookup maps
      frame_to_frameID_lookup_[target_key] = tf_target_id;
      frameID_to_frame_lookup_.vec.push_back(target_key);

      // create new node and add it to frameID_to_node_lookup_ map
      tf_target_node = new TFTreeNode ();
      tf_target_node->nodeID_ = tf_target_id;
      tf_target_node->parent_node_ = tf_source_node;
      tf_target_node->parent_nodeID_ = tf_source_id;
      frameID_to_node_lookup_.push_back(tf_target_node);

      dirty_frameID_table_ = true;
    }

    assert (tf_source_node && tf_target_node);

    // updating node
    tf_source_node->subnodes_.insert (tf_target_node);
    tf_target_node->tf_msg_ = geometry_msgs::TransformStampedPtr(new geometry_msgs::TransformStamped(msg));
    tf_target_node->changed_ = true;
    tf_target_node->last_received_ = ros::Time::now ();
    tf_target_node->update_counter_++;

    tf_nodes_with_parents_.insert (tf_target_id);
  }

  void TFTree::searchForRootNodes ()
  {
    size_t i;

    boost::mutex::scoped_lock lock (mutex_);

    tf_root_nodes_.clear ();
    tf_root_nodes_.reserve (tf_nodes_with_parents_.size ());

    // search for nodes without parents
    for (i = 0; i < frame_count_; ++i)
      if (tf_nodes_with_parents_.find (i) == tf_nodes_with_parents_.end ())
        tf_root_nodes_.push_back (i);
  }

  void TFTree::showTFTree (bool verbose)
  {
    searchForRootNodes ();
    for (size_t i = 0; i < tf_root_nodes_.size (); ++i)
    {
      showTFTree (tf_root_nodes_[i], verbose);
    }
  }

  void TFTree::deleteTree ()
  {
    size_t i;
    map<unsigned int, TFTreeNode*>::iterator it;

    boost::mutex::scoped_lock lock (mutex_);

    for (i = 0; i < frame_count_; ++i)
        delete frameID_to_node_lookup_[i];
  }

  void TFTree::showTFTree (const string& root, bool verbose)
  {

    // empty root node - show full tree
    if (root.empty ())
    {
      showTFTree (verbose);
      return;
    }

    string root_node = root;
    map<string, unsigned int>::iterator it;

    // search for root node string
    it = frame_to_frameID_lookup_.find (root_node);
    if (it != frame_to_frameID_lookup_.end ())
    {
      showTFTree (it->second, verbose);
      return;
    }

    // search for root node string - remove trailing "/"
    it = frame_to_frameID_lookup_.find ("/" + root_node);
    if (it != frame_to_frameID_lookup_.end ())
    {
      showTFTree (it->second, verbose);
      return;
    }

    {
      cout << "Root not found: " << root << endl;
    }

  }

  void TFTree::showTFTree (unsigned int root, bool verbose)
  {
    boost::mutex::scoped_lock lock (mutex_);

    string node_name;
    map<unsigned int, TFTreeNode*>::iterator it;

    if (root<frameID_to_node_lookup_.size())
    {
      TFTreeIterator tf_it (frameID_to_node_lookup_[root]);

      vector<bool> tree_connector;
      tree_connector.reserve (frame_count_);

      if (*tf_it && (tf_it.getBaseTFFrame ()<frameID_to_frame_lookup_.vec.size()))
      {
        // get node string
        node_name = frameID_to_frame_lookup_.vec[tf_it.getBaseTFFrame ()];
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
        node_name = frameID_to_frame_lookup_.vec[tf_it.getTargetTFFrame ()];

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
        //eol
        cout << endl;

        // increase tree iterator
        ++tf_it;
      }

    }
  }

}
