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

#include <map>
#include <stack>
#include <iterator>

using namespace std;

namespace tf_tools
{

class TFTreeNode
{
public:
  TFTreeNode() :
      changed_(false), max_delay_(0.0f), update_counter_(0)
  {
    first_received_ = ros::Time::now();
  }
  ~TFTreeNode()
  {
  }

  float getAvgDelay()
  {
    float avg_delay = ros::Duration(last_received_ - tf_msg_.header.stamp).toSec();
    max_delay_ = max<float>(max_delay_, abs(avg_delay));
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


  geometry_msgs::TransformStamped tf_msg_;
  bool changed_;
  ros::Time last_received_;
  ros::Time first_received_;
  float max_delay_;
  unsigned int update_counter_;

  map<string, TFTreeNode*> subnodes_;
};

class TFTree
{
public:
  TFTree()
  {
  }
  virtual ~TFTree()
  {
    deleteTree();
  }

  void addTFMessage(const geometry_msgs::TransformStamped& msg);
  void searchForRootNodes();
  void showTFTree(bool verbose = false)
  {
    cout << "TF-Tree: "<< endl;

    searchForRootNodes();
    for (size_t i = 0; i < tf_root_nodes_.size(); ++i)
    {
      showTFTree(tf_root_nodes_[i], verbose);
    }

  }

  void findTFPath(const string& from_TF, const string& to_TF, vector<pair<string, TFTreeNode*> >& tf_path)
  {
    map<string, TFTreeNode*>::iterator from_it;
    map<string, TFTreeNode*>::iterator to_it;
    map<string, TFTreeNode*>::iterator it;

    from_it = tf_tree_nodes_.find(from_TF);
    to_it = tf_tree_nodes_.find(to_TF);

    pair<string, TFTreeNode*> result;

    while ((to_it!=tf_tree_nodes_.end()) && (to_it!=from_it))
    {
      const string& base_key = to_it->second->tf_msg_.header.frame_id;
      result.first = base_key;
      result.second = to_it->second;
      tf_path.push_back(result);

      to_it = tf_tree_nodes_.find(base_key);
    }

  }

  void showTFTree(const string& root, bool verbose = true);

  void showTFPath(const string& from_TF, const string& to_TF, bool verbose = true)
  {
    vector<pair<string, TFTreeNode*> > tf_path;

    findTFPath(from_TF, to_TF, tf_path);

    size_t path_len = tf_path.size();

    cout << "TF-Path from "<< from_TF<< " to " <<to_TF<<endl;

    for (size_t i=0; i<path_len; ++i)
    {
      std::cout<<tf_path.back().first;
      if (verbose)
      {
        TFTreeNode* node = tf_path.back().second;
        printf(" [Average Delay: %.4fs, Max Delay: %.4fs, Freq: %.4fHz]", node->getAvgDelay()
                                                                        , node->getMaxDelay()
                                                                        , node->getFrequency());
      }
      tf_path.pop_back();
      cout<< endl;
    }

  }

  void deleteTree ()
  {
    map<string, TFTreeNode*>::iterator it;
    map<string, TFTreeNode*>::iterator it_end = tf_tree_nodes_.end();

    for (it = tf_tree_nodes_.begin(); it != it_end; ++it)
      delete it->second;
  }

protected:
  map<string, TFTreeNode*> tf_tree_nodes_;

  set<string> tf_full_nodes_set_;
  set<string> tf_nodes_with_parents_;

  vector<string> tf_root_nodes_;
};

class TFTreeIterator : iterator<forward_iterator_tag, TFTreeNode>
{
public:
  /// Default ctor, only used for the end-iterator
  TFTreeIterator()
  {
  }


  TFTreeIterator(TFTreeNode* node)
  {
    StackElement s;
    s.depth = 0;
    s.node = node;
    s.last_child = true;
    stack.push(s);

    singleIncrement();
  }



  TFTreeIterator(const TFTreeIterator& other) :
      stack(other.stack)
  {

  }

  const TFTreeNode* operator->() const
  {
    return getNode();
  }

  TFTreeNode* operator->()
  {
    return getNode();
  }

  const TFTreeNode* operator*() const
  {
    return getNode();
  }

  TFTreeNode* operator*()
  {
    return getNode();
  }

  /// postfix increment operator of iterator (it++)
  TFTreeIterator operator++(int)
  {
    TFTreeIterator result = *this;
    ++(*this);
    return result;
  }

  /// Prefix increment operator to advance the iterator
  TFTreeIterator& operator++()
  {

    if (!this->stack.empty())
    {
      this->singleIncrement();
    }

    return *this;
  }

  const string& getTargetTFFrame()
  {
    return stack.top().node->tf_msg_.child_frame_id;
  }
  const string& getBaseTFFrame()
  {
    return stack.top().node->tf_msg_.header.frame_id;
  }

  bool isLastChild()
  {
    return stack.top().last_child;
  }

  unsigned char getDepth()
  {
    return stack.top().depth;
  }

protected:
  inline TFTreeNode* getNode() const
  {
    if (!this->stack.empty())
    {
      return stack.top().node;
    }
    else
    {
      return 0;
    }
  }

  void singleIncrement()
  {
    StackElement top = stack.top();
    stack.pop();

    StackElement s;
    s.depth = top.depth + 1;

    map<string, TFTreeNode*>::iterator it;
    map<string, TFTreeNode*>::iterator it_end = top.node->subnodes_.end();

    size_t subnodes = 0;
    for (it = top.node->subnodes_.begin(); it != it_end; ++it)
    {
      s.last_child = (++subnodes==1);
      s.node = it->second;
      stack.push(s);
    }

  }

  /// Element on the internal recursion stack of the iterator
  struct StackElement
  {
    unsigned char depth;
    TFTreeNode* node;
    bool last_child;
  };

  /// Internal recursion stack. Apparently a stack of vector works fastest here.
  std::stack<StackElement, std::vector<StackElement> > stack;

};

}

#endif /* TFTREE_H_ */
