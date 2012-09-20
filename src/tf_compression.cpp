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
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/flush.hpp>

#include <iostream>

#include "tf_compression.h"

#include <zlib.h> # for MAX_WBITS

namespace tf_tools
{

  bool TFCompression::hasTFNodeChanged(TFTreeNode* node)
  {
    if (!node->tf_msg_)
      return false;
    if (!node->tf_msg_sent_)
      return true;

    tf::Transform tf1, tf2;
    tf::transformMsgToTF(node->tf_msg_->transform, tf1);
    tf::transformMsgToTF(node->tf_msg_sent_->transform, tf2);

    if (linear_change_threshold_ == 0.0 || angular_change_threshold_ == 0.0 ||
        tf1.getOrigin().distance(tf2.getOrigin()) > linear_change_threshold_ ||
        tf1.getRotation().angle(tf2.getRotation()) > angular_change_threshold_) return true;
    return false;

  }


  void TFCompression::encodeCompressedTFStream (std::ostream& compressedDataOut_arg)
{
    boost::mutex::scoped_lock lock(mutex_);

    std::stringstream uncompressedData;
    boost::archive::binary_oarchive oa(uncompressedData, boost::archive::no_header);

    if (ros::Duration(ros::Time::now()-last_frame_table_transmission_).toSec()>min_update_rate_)
      dirty_frameID_table_ = true;

    if (dirty_frameID_table_)
    {
      dirty_frameID_table_ = false;
      last_frame_table_transmission_ = ros::Time::now();

      FrameHeader header(FrameHeader::COMPRESSED_FRAME_ID_TABLE);

      oa << header;
      {
       // boost::mutex::scoped_lock lock(mutex_);
        oa << frameID_to_frame_lookup_;

       // printFrameIDTable();

      }
    }

    // encode all nodes

    {
      updateContainer_.clear();
      updateContainer_.reserve(frameID_to_node_lookup_.size());

      bool TFMsgChanged = false;
      {
       // boost::mutex::scoped_lock lock(mutex_);

        vector<TFTreeNode*>::iterator it;;
        vector<TFTreeNode*>::iterator it_end = frameID_to_node_lookup_.end();
        for (it = frameID_to_node_lookup_.begin(); it!=it_end; ++it)
        {
          if (1)//hasTFNodeChanged(*it))
          {
            TFMsgChanged = true;
            updateContainer_.add(*it);
          }
        }
      }

      if (TFMsgChanged)
      {
        FrameHeader header(FrameHeader::COMPRESSED_TF_CONTAINER);
        oa << header;
        oa << updateContainer_;
      }
    }
    // zlib compression

    uncompressedData.flush();

    if (uncompressedData.str().length()>0)
    {
      try
      {
        boost::iostreams::zlib_params p;
        p.window_bits = 16 +  MAX_WBITS;

        boost::iostreams::filtering_streambuf<boost::iostreams::output> out;
        out.push(boost::iostreams::zlib_compressor(p));
        out.push(compressedDataOut_arg);
        boost::iostreams::copy(uncompressedData, out);
        compressedDataOut_arg.flush();

      } catch (boost::iostreams::zlib_error& e)
      {
        ROS_ERROR("ZLIB encoding failed: %s", e.what());
      }
    }
}

  void TFCompression::decodeCompressedTFStream (std::istream& compressedDataIn_arg,
                                                tf::tfMessage& decoded_msg)
{
    size_t i;
    boost::mutex::scoped_lock lock(mutex_);

    std::stringstream uncompressedData;

    // zlib decompression
    try
    {
      boost::iostreams::zlib_params p;
      p.window_bits = 16 +  MAX_WBITS;

      boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
      in.push(boost::iostreams::zlib_decompressor(p));
      in.push(compressedDataIn_arg);
      boost::iostreams::copy(in, uncompressedData);

    } catch (boost::iostreams::zlib_error& e) {
      ROS_ERROR("ZLIB decoding failed: %s (%d)", e.what(), e.error());

      if (e.error() == boost::iostreams::zlib::stream_end)
      {
        cout << "Stream end zlib error";
      }
      else if (e.error() == boost::iostreams::zlib::stream_error)
      {
        cout << "Stream zlib error";
      }
      else if (e.error() == boost::iostreams::zlib::version_error)
      {
        cout << "Version zlib error";
      }
      else if (e.error() == boost::iostreams::zlib::data_error)
      {
        cout << "Data zlib error";
      }
      else if (e.error() == boost::iostreams::zlib::mem_error)
      {
        cout << "Memory zlib error";
      }
      else if (e.error() == boost::iostreams::zlib::buf_error)
      {
        cout << "Buffer zlib error";
      }
      else
      {
        cout << "Unknown zlib error";
      }
      cout<<endl;
    }

    // deserialize data

    boost::archive::binary_iarchive ia(uncompressedData, boost::archive::no_header);

    FrameHeader header;

    try
    {

      while (!uncompressedData.eof())
      {
        ia >> header;
        switch (header.type_)
        {
          case FrameHeader::COMPRESSED_FRAME_ID_TABLE:
            ia >> frameID_to_frame_lookup_;

            {
              size_t i;
              std::cout<< "FrameTable" << std::endl;

              for (i=0; i<frameID_to_frame_lookup_.size(); ++i)
                std::cout<<frameID_to_frame_lookup_[i]<< std::endl;
            }
            break;
          case FrameHeader::COMPRESSED_TF_CONTAINER:
            {
              geometry_msgs::TransformStamped tf;

              uint16_t source_frame_id, target_frame_id;
              size_t table_size = frameID_to_frame_lookup_.size();

              updateContainer_.clear();
              ia >> updateContainer_;

              for (i=0; i<updateContainer_.size(); ++i)
              {
                updateContainer_.decode(i, tf, source_frame_id, target_frame_id);

                if ( (source_frame_id<table_size) && (target_frame_id<table_size))
                {
                  tf.header.frame_id = frameID_to_frame_lookup_[source_frame_id];
                  tf.child_frame_id = frameID_to_frame_lookup_[target_frame_id];

                  decoded_msg.transforms.push_back(tf);

                  std::cout<< tf.header.frame_id << "--" << tf.child_frame_id << std::endl;
                }

                // PUBLISH MESSAGE
              }
            }
            break;
          default:
            break;
        }
      }
    } catch (boost::archive::archive_exception& e) {}

}
/*
  void TFCompression::encodeIntraUpdate (std::ostream& compressedDataOut_arg)
  {

    searchForRootNodes ();
    for (size_t i = 0; i < tf_root_nodes_.size (); ++i)
    {
      encodeIntraUpdate (compressedDataOut_arg, tf_root_nodes_[i]);
    }
  }
  void TFCompression::encodeIntraUpdate (std::ostream& compressedDataOut_arg, unsigned int root)
  {

    boost::mutex::scoped_lock lock (mutex_);

    string node_name;
    map<unsigned int, TFTreeNode*>::iterator it;

    if (root<frameID_to_node_lookup_.size())
    {
      updateContainer_.clear();
      updateContainer_.reserve(frame_count_);

      TFTreeIterator tf_it (frameID_to_node_lookup_[root]);

      while (*tf_it)
      {
        geometry_msgs::TransformStampedConstPtr tf_message = tf_it.getTFMessage();

        updateContainer_.encode(tf_it.getTFMessage(), tf_it.getBaseTFFrame(), tf_it.getTargetTFFrame() );

        // increase tree iterator
        ++tf_it;
      }

      boost::archive::binary_oarchive oa(compressedDataOut_arg, boost::archive::no_header);
      oa << updateContainer_;
    }

  }
  void TFCompression::encodeIntraUpdate (std::ostream& compressedDataOut_arg, const string& root)
  {
    // empty root node - show full tree
    if (root.empty ())
    {
      encodeIntraUpdate (compressedDataOut_arg);
      return;
    }

    string root_node = root;
    map<string, unsigned int>::iterator it;

    // search for root node string
    it = frame_to_frameID_lookup_.find (root_node);
    if (it != frame_to_frameID_lookup_.end ())
    {
      encodeIntraUpdate (compressedDataOut_arg, it->second);
      return;
    }

    // search for root node string - remove trailing "/"
    it = frame_to_frameID_lookup_.find ("/" + root_node);
    if (it != frame_to_frameID_lookup_.end ())
    {
      encodeIntraUpdate (compressedDataOut_arg, it->second);
      return;
    }

    {
      cout << "Root not found: " << root << endl;
    }
  }
*/
}
