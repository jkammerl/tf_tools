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

#ifndef TF_COMPRESSION_H_
#define TF_COMPRESSION_H_

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

#include "tf_tree.h"
#include "tf_tree_iterator.h"
#include "tf_container.h"

namespace tf_tunnel
{

  class TFCompression: public TFTree
  {
    public:
      TFCompression () :
        TFTree(),
        intra_update_rate_(5.0),
        linear_change_threshold_(1.0),
        angular_change_threshold_(0.02)
      { }

      virtual ~TFCompression () { }

      void encodeCompressedTFStream (std::ostream& compressedDataOut_arg,
                                     const std::vector<std::string>& frame_filter_);

      void decodeCompressedTFStream (std::istream& compressedDataIn_arg,
                                     tf::tfMessage& decoded_msg);

      void setAngularThreshold(double thres)
      {
        angular_change_threshold_ = thres;
      }

      void setLinearThreshold(double thres)
      {
        linear_change_threshold_ = thres;
      }

    protected:

      bool hasTFNodeChanged(TFTreeNode* node);
      bool intraUpdateRequired(TFTreeNode* node);

      double intra_update_rate_;

      ros::Time last_frame_table_transmission_;

      double linear_change_threshold_;
      double angular_change_threshold_;

      TFMessageContainer TFContainer_;

  };

}

#endif /* TF_COMPRESSION_H_ */
