#include "batch_processor.h"
#include "string"
#include <QString>

using namespace std;

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "batch_cloud_merge_node");
    ros::NodeHandle n;

    ros::NodeHandle aRosNode("~");

    BatchProcessor<pcl::PointXYZRGB> aBatchProcessorNode(aRosNode);

    string file;
    bool found = aRosNode.getParam("input_pcd",file);

    for (int i=0; i<1008; i++)
    {
        QString filename;
        filename.sprintf("/media/FA3407FA3407B8A1/longterm/4/%05d.pcd",i);
//        ROS_INFO_STREAM("Filename "<<filename.toStdString());
        aBatchProcessorNode.loadPCDFile(filename.toStdString().c_str());
    }


//    static bool process = true;

//    ros::Rate loop_rate(10);
//    while (ros::ok())
//    {
//        ros::spinOnce();
//        loop_rate.sleep();
//        if (process){
//            process = false;
//            aBatchProcessorNode.loadPCDFile(file.c_str());
//        }
//    }
}
