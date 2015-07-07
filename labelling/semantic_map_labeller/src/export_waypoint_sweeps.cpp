#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vector>
#include <QDir>



#include "load_utilities.h"
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
   if (argc == 4)
   {
      from=argv[1];
      to=argv[2];
      waypId = argv[3];
      cout<<"Exporting from "<<from<<" to "<<to<<" waypoint "<<waypId<<endl;
   } else {
      cout<<"Please provide suitable arguments"<<endl;
      return -1;
   }

   vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(from, waypId);
   ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());


   string output_file=to+"/"+waypId+".txt";
   ofstream to_fstream;
   to_fstream.open(output_file);
   SemanticRoomXMLParser<PointType> parser(to+"/");
   for (size_t i=0; i</*matchingObservations.size()*/10;i++)
   {
      ROS_INFO_STREAM("Parsing "<<matchingObservations[i]);
      SemanticRoom<PointType> room = SemanticRoomXMLParser<PointType>::loadRoomFromXML(matchingObservations[i],true);
      string output_xml = parser.saveRoomAsXML(room);
      to_fstream<<output_xml<<endl;
   }

}
