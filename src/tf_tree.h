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
#include <set>
#include <ostream>
#include <stdint.h>

#include "tf_node.h"
#include "tf_tree_iterator.h"

using namespace std;

namespace tf_tools
{
  class FrameTable
  {
      friend class boost::serialization::access;
      friend std::ostream & operator<<(std::ostream &os, const FrameTable &gp);

      template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
          ar & vec;
      }

  public:
      FrameTable() { }
      virtual ~FrameTable(){}

      size_t size()
      {
        return vec.size();
      }

      string& operator [](const size_t& idx)
      {
        assert(idx<vec.size());

        return vec[idx];
      }

      vector<string> vec;

  };

  class TFTree
  {
    public:
      TFTree () : frame_count_ (0), dirty_frameID_table_(false)
      {
        reset ();
      }
      virtual ~TFTree ()
      {
        deleteTree ();
      }

      void reset ()
      {
        deleteTree ();

        frame_to_frameID_lookup_.clear ();
        frameID_to_frame_lookup_.vec.clear ();
        frameID_to_node_lookup_.clear ();
        tf_nodes_with_parents_.clear ();
        tf_root_nodes_.clear ();
      }

      void addTFMessage (const geometry_msgs::TransformStamped& msg);

      void searchForRootNodes ();

      void showTFTree (bool verbose = false);
      void showTFTree (unsigned int root, bool verbose = true);
      void showTFTree (const string& root, bool verbose = true);

      void deleteTree ();

      void printFrameIDTable()
      {
        size_t i;
        for (i=0; i<frameID_to_frame_lookup_.size(); ++i)
        {
          std::cout << "ID:" <<i << frameID_to_frame_lookup_[i] << std::endl;
        }
      }

    protected:
      boost::mutex mutex_;

      // frame  <-> frameID lookup maps
      map<string, unsigned int> frame_to_frameID_lookup_;
      FrameTable frameID_to_frame_lookup_;

      // frameID -> node lookup map
      vector<TFTreeNode*> frameID_to_node_lookup_;

      // amount of frames/nodes
      unsigned int frame_count_;

      // set describing nodes that have a parent
      set<unsigned int> tf_nodes_with_parents_;

      // vector of root node (IDs)
      vector<unsigned int> tf_root_nodes_;

      // dirty flag for frame ID table
      bool dirty_frameID_table_;
  };

}

#endif /* TFTREE_H_ */
