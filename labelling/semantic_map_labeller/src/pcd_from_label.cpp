#include <vector>
#include <QDir>

#include <metaroom_xml_parser/load_utilities.h>
#include "labeller.h"

typedef pcl::PointXYZRGB PointType;

using namespace std;

int main(int argc, char** argv)
{
   string waypId = "";
   string path = "";
   if (argc == 3)
   {
       path = argv[1];
      waypId = argv[2];
   } else {
      cout<<"Please provide path and waypoint as argument"<<endl;
      return -1;
   }


   vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(path, waypId);
   ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());

   for (size_t i=0; i<matchingObservations.size();i++)
   {

      ROS_INFO_STREAM("Parsing "<<matchingObservations[i]);

      // find label image files in this folder
      unsigned found = matchingObservations[i].find_last_of("/");
      std::string base_path = matchingObservations[i].substr(0,found+1);
//      QDir sweepFolder(matchingObservations[i]);
      QStringList labelFiles = QDir(base_path.c_str()).entryList(QStringList("*label*.jpg"));
      cout<<"Found "<<labelFiles.size()<<" labels"<<endl;

      for (auto file : labelFiles)
      {
         string completeFile = base_path + file.toStdString();
         cout<<completeFile<<endl;
         create_point_cloud_from_label(completeFile, matchingObservations[i]);
      }

//      return;
   }
}
