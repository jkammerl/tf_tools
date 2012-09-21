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

#ifndef TFTREE_ITERATOR_H_
#define TFTREE_ITERATOR_H_

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>

#include <map>
#include <set>
#include <stack>
#include <iterator>

#include "tf_node.h"

namespace tf_tunnel
{
  class TFTreeIterator : std::iterator<std::forward_iterator_tag, TFTreeNode>
  {
    public:
      /// Default ctor, only used for the end-iterator
      TFTreeIterator ()
      {
      }

      TFTreeIterator (TFTreeNode* node)
      {
        StackElement s;
        s.depth = 0;
        s.node = node;
        s.last_child = true;
        stack.push (s);

        singleIncrement ();
      }

      TFTreeIterator (const TFTreeIterator& other) :
          stack (other.stack)
      {

      }

      const TFTreeNode* operator-> () const
      {
        return getNode ();
      }

      TFTreeNode* operator-> ()
      {
        return getNode ();
      }

      const TFTreeNode* operator* () const
      {
        return getNode ();
      }

      TFTreeNode* operator* ()
      {
        return getNode ();
      }

      /// postfix increment operator of iterator (it++)
      TFTreeIterator operator++ (int)
      {
        TFTreeIterator result = *this;
        ++ (*this);
        return result;
      }

      /// Prefix increment operator to advance the iterator
      TFTreeIterator& operator++ ()
      {

        if (!this->stack.empty ())
        {
          this->singleIncrement ();
        }

        return *this;
      }

      const unsigned int& getTargetTFFrame ()
      {
        return stack.top ().node->nodeID_;
      }

      const unsigned int& getBaseTFFrame ()
      {
        return stack.top ().node->parent_nodeID_;
      }

      geometry_msgs::TransformStampedConstPtr getTFMessage()
      {
        return stack.top ().node->tf_msg_;
      }

      bool isLastChild ()
      {
        return stack.top ().last_child;
      }

      unsigned char getDepth ()
      {
        return stack.top ().depth;
      }

    protected:
      inline TFTreeNode* getNode () const
      {
        if (!this->stack.empty ())
        {
          return stack.top ().node;
        }
        else
        {
          return 0;
        }
      }

      void singleIncrement ()
      {
        StackElement top = stack.top ();
        stack.pop ();

        StackElement s;
        s.depth = top.depth + 1;

        // iterators
        std::set<TFTreeNode*>::iterator it;
        std::set<TFTreeNode*>::const_iterator it_end = top.node->subnodes_.end ();

        size_t subnodes = 0;
        for (it = top.node->subnodes_.begin (); it != it_end; ++it)
        {
          s.last_child = (++subnodes == 1);
          s.node = *it;
          stack.push (s);
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
