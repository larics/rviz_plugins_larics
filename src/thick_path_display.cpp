/*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
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
*/

#include <boost/bind.hpp>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMatrix4.h>
#include <tf/transform_listener.h>
#include "rviz/default_plugin/point_cloud_common.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/validate_floats.h"
#include "thick_path_display.h"
#include "geometry_msgs/PoseStamped.h"


namespace rviz 
{

ThickPathDisplay::ThickPathDisplay() : point_cloud_common_( new PointCloudCommon(this))
{

    buffer_length_property_ = new IntProperty( "Buffer Length", 1,
                                               "Number of paths to display.",
                                               this, SLOT( updateQueueSize()));
    buffer_length_property_->setMin( 1 );
    update_nh_.setCallbackQueue(point_cloud_common_->getCallbackQueue());
}

ThickPathDisplay::~ThickPathDisplay()
{
    delete point_cloud_common_;
}

void ThickPathDisplay::onInitialize()
{
    MFDClass::onInitialize();
    point_cloud_common_->initialize(context_, scene_node_);
}


void ThickPathDisplay::updateQueueSize()
{
    tf_filter_->setQueueSize((uint32_t) buffer_length_property_->getInt());
}

bool validateFloats( const nav_msgs::Path& msg )
{
	bool valid = true;
	valid = valid && validateFloats( msg.poses );
	return valid;
}

void ThickPathDisplay::processMessage( const nav_msgs::Path::ConstPtr& msg )
{
    if( !validateFloats( *msg ))
	{
		setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
		return;
	}

    sensor_msgs::PointCloudPtr cloud(new sensor_msgs::PointCloud);
    cloud->header = msg->header;
    cloud->points.resize(msg->poses.size());

    for(int i=0; i<msg->poses.size(); i++) {
        cloud->points[i].x = msg->poses[i].pose.position.x;
        cloud->points[i].y = msg->poses[i].pose.position.y;
        cloud->points[i].z = msg->poses[i].pose.position.z;
    }
    point_cloud_common_->addMessage(cloud);
}

void ThickPathDisplay::update(float wall_dt, float ros_dt) {
    point_cloud_common_->update(wall_dt, ros_dt);
}

void ThickPathDisplay::reset()
{
    MFDClass::reset();
    point_cloud_common_->reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::ThickPathDisplay, rviz::Display )
