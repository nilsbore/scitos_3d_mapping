#include <pcl/io/pcd_io.h>
#include <vector>
#include <QDir>

#include <metaroom_xml_parser/load_utilities.h>
#include <semantic_map/room_utilities.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;


using namespace std;

int main(int argc, char** argv)
{
   string waypId = "";
   string from="";
   if (argc == 3)
   {
      from=argv[1];
      waypId = argv[2];
      cout<<"Rebuilding merged clouds from "<<from<<" waypoint "<<waypId<<endl;
   } else if (argc==2) {
      from=argv[1];
      cout<<"Rebuilding merged clouds from "<<from<<endl;

   }else {
      cout<<"Please provide arguments: path [waypoint]"<<endl;
      return -1;
   }

   vector<string> matchingObservations;
   if (waypId != ""){
   	matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(from, waypId);
   	ROS_INFO_STREAM("Observations for waypoint "<<matchingObservations.size());
   } else {
   	matchingObservations = semantic_map_load_utilties::getSweepXmls<PointType>(from);
   	ROS_INFO_STREAM("Observations "<<matchingObservations.size());
  }


   SemanticRoomXMLParser<PointType> parser(from+"/");
   for (size_t i=0; i<matchingObservations.size();i++)
   {
      ROS_INFO_STREAM("Parsing "<<matchingObservations[i]);
      SemanticRoom<PointType> room = SemanticRoomXMLParser<PointType>::loadRoomFromXML(matchingObservations[i],true);
      bool rebuilt = semantic_map_room_utilities::rebuildRegisteredCloud<PointType>(room);
      if (!rebuilt){
	      rebuilt = semantic_map_room_utilities::rebuildOriginalCloud<PointType>(room);
	      if (rebuilt) {
		parser.saveRoomAsXML(room);
	      }
      } else {
	      auto origTransforms = room.getIntermediateCloudTransforms();
	      tf::StampedTransform origin = origTransforms[0];
	      CloudPtr completeCloud = room.getCompleteRoomCloud();
	      pcl_ros::transformPointCloud(*completeCloud, *completeCloud,origin);
	      room.setCompleteRoomCloud(completeCloud);
	      string output_xml = parser.saveRoomAsXML(room);
      }
   }

}
