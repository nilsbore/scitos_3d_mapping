#ifndef __BATCH_PROCESSOR__H
#define __BATCH_PROCESSOR__H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

#include <ros_datacentre/message_store.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <semantic_map/RoomObservation.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "cloud_merge.h"
#include "room.h"
#include "roomXMLparser.h"
#include "semanticMapSummaryParser.h"
#include "cloud_merge_node.h"

template <class PointType>
class BatchProcessor {
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::iterator CloudIterator;


    BatchProcessor(ros::NodeHandle nh)
    {
        ROS_INFO_STREAM("Batch cloud merge node initialized");

        m_NodeHandle = nh;
        m_PublisherMergedCloud = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/merged_point_cloud", 1);
        m_PublisherRoomObservation = m_NodeHandle.advertise<semantic_map::RoomObservation>("/local_metric_map/room_observations", 1000);
    }

    ~BatchProcessor(){}

    void loadPCDFile(char* file);

private:
    CloudPtr                                                                    m_IntermediateCloud;
    CloudPtr                                                                    m_MergedCloud;
    double                                                                      m_dMaximumPointDistance;
    ros::Publisher                                                              m_PublisherMergedCloud;
    ros::Publisher                                                              m_PublisherRoomObservation;
    ros::NodeHandle                                                             m_NodeHandle;


};

template <class PointType>
void BatchProcessor<PointType>::loadPCDFile(char* file)
{
    // create an observation based on a saved point cloud
    SemanticRoom<PointType>  aSemanticRoom;

    pcl::PCDReader reader;
    CloudPtr cloud (new Cloud);
    reader.read (file, *cloud);       

    // subsample
    CloudPtr subsampledCloud (new Cloud);
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.05,0.05,0.05);
    vg.filter (*subsampledCloud);

    aSemanticRoom.setCompleteRoomCloud(subsampledCloud);

    aSemanticRoom.setRoomLogStartTime(ros::Time::now().toBoost());

    int roomId, runNumber;
    CloudMergeNode<PointType>::findSemanticRoomIDAndLogName(aSemanticRoom, roomId, runNumber);
    ROS_INFO_STREAM("Found id and run number");

    std::string logRunName = "patrol_run_";
    logRunName += QString::number(runNumber).toStdString();

    static std::locale loc(std::cout.getloc(),
                           new boost::posix_time::time_facet("%Y%m%d"));

    std::basic_stringstream<char> wss;
    wss.imbue(loc);
    wss << ros::Time::now().toBoost();
    std::string logName = wss.str()+"_"+logRunName;
    ROS_INFO_STREAM("Log name set to "<<logName);

    aSemanticRoom.setRoomRunNumber(roomId);
    aSemanticRoom.setRoomLogName(logName);
    aSemanticRoom.setRoomLogEndTime(ros::Time::now().toBoost());

    SemanticRoomXMLParser<PointType> parser;
    ROS_INFO_STREAM("Saving semantic room file");
    std::string roomXMLPath = parser.saveRoomAsXML(aSemanticRoom);
    ROS_INFO_STREAM("Saved semantic room");

    // Pulbish room observation
    semantic_map::RoomObservation obs_msg;
    obs_msg.xml_file_name = roomXMLPath;

    m_PublisherRoomObservation.publish(obs_msg);

    sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(*cloud, msg_cloud);
    m_PublisherMergedCloud.publish(msg_cloud);
}


#endif
