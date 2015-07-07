#include <vector>

#include "load_utilities.h"
#include "labeller.h"

typedef pcl::PointXYZRGB PointType;

using namespace std;

int main(int argc, char** argv)
{
    string path="";
   string waypId = "";
   if (argc == 3)
   {
       path=argv[1];
      waypId = argv[2];
   } else {
      cout<<"Please provide path and waypoint as argument"<<endl;
      return -1;
   }

   ofstream out;
   out.open("sweeps.txt");

   vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(path, waypId);
   ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());

   for (size_t i=0; i<matchingObservations.size();i++)
   {
      ROS_INFO_STREAM("Parsing "<<matchingObservations[i]);
      create_int_images(matchingObservations[i]);
      out<<matchingObservations[i]<<endl;
   }

   out.close();
}
