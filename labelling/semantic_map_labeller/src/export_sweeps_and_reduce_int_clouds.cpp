#include <pcl/io/pcd_io.h>
#include <vector>
#include <QDir>

#include <metaroom_xml_parser/load_utilities.h>
#include "labeller.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;


using namespace std;

int main(int argc, char** argv)
{
    string waypId = "";
    string from="";
    string to="";
    int intermediate_clouds;
    if (argc == 5)
    {
        from=argv[1];
        to=argv[2];
        waypId = argv[3];
        intermediate_clouds = argv[4];
        cout<<"Exporting from "<<from<<" to "<<to<<" waypoint "<<waypId<<" and keeping "<<intermediate_clouds<<" interemdiate clouds "<<endl;
    } else {
        cout<<"Please provide suitable arguments"<<endl;
        return -1;
    }

    vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(from, waypId);
    ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());


    SemanticRoomXMLParser<PointType> parser(to+"/");
    for (size_t i=0; i<matchingObservations.size();i++)
    {
        ROS_INFO_STREAM("Parsing "<<matchingObservations[i]);
        SemanticRoom<PointType> room = SemanticRoomXMLParser<PointType>::loadRoomFromXML(matchingObservations[i],true);
        std::vector<tf::StampedTransform> tr_reg = room.getIntermediateCloudTransformsRegistered();
        std::vector<tf::StampedTransform> tr = room.getIntermediateCloudTransforms();
        std::vector<image_geometry::PinholeCameraModel> cam = room.getIntermediateCloudCameraParameters();
        std::vector<image_geometry::PinholeCameraModel> cam_reg = room.getIntermediateCloudCameraParametersCorrected();
        std::vector<CloudPtr> cl = room.getIntermediateClouds();

        if (intermediate_clouds < cl.size())
        {
            tr_reg.resize(intermediate_clouds);
            tr.resize(intermediate_clouds);
            cam.resize(intermediate_clouds);
            cam_reg.resize(intermediate_clouds);
            cl.resize(intermediate_clouds);

            room.clearIntermediateCloudCameraParametersCorrected();
            room.clearIntermediateCloudRegisteredTransforms();
            room.clearIntermediateClouds();

            for (size_t i=0; i<cl.size();i++){
                room.addIntermediateRoomCloud(cl[i],tr[i],cam[i]);
                room.addIntermediateRoomCloudRegisteredTransform(tr_reg[i]);
                room.addIntermediateCloudCameraParametersCorrected(cam_reg[i]);
            }
        }

        string output_xml = parser.saveRoomAsXML(room);
    }

}
