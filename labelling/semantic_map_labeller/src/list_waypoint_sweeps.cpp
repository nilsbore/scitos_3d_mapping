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
   if (argc == 3)
   {
      from=argv[1];
      waypId = argv[2];
      cout<<"Listing sweeps from "<<from<<" waypoint "<<waypId<<endl;
   } else {
      cout<<"Please provide suitable arguments"<<endl;
      return -1;
   }

   vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(from, waypId);
   ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());


   string output_file=from+"/"+waypId+".txt";

   cout<<"Listing sweeps in "<<output_file<<endl;
   ofstream to_fstream;
   to_fstream.open(output_file);
   for (size_t i=0; i<matchingObservations.size();i++)
   {
      cout<<"Found: "<<matchingObservations[i]<<endl;
      to_fstream<<matchingObservations[i]<<endl;
   }

}
