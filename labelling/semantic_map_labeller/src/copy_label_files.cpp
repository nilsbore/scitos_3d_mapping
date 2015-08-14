#include <vector>
#include <QDir>

#include <metaroom_xml_parser/load_utilities.h>
#include "labeller.h"

typedef pcl::PointXYZRGB PointType;

using namespace std;

string getLogName(string sweep_path)
{
   int index = sweep_path.find("patrol_run");
   return sweep_path.substr(index-9,30);
}

int main(int argc, char** argv)
{
   string path_1, path_2;
   string waypId = "";
   if (argc == 4)
   {
      waypId = argv[1];
      path_1 = argv[2];
      path_2 = argv[3];
   } else {
      cout<<"Please provide waypoint and the path for two semantic map locations (from and to)"<<endl;
      return -1;
   }


   vector<string> matchingObservations_from = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(path_1, waypId);
   ROS_INFO_STREAM("Observation matches for waypoint (semantic map from) "<<matchingObservations_from.size());

   vector<string> matchingObservations_to = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(path_2, waypId);
   ROS_INFO_STREAM("Observation matches for waypoint (semantic map to) "<<matchingObservations_to.size());

   for (auto sweep_path : matchingObservations_from)
   {
      string from_path = getLogName(sweep_path);
      auto it = std::find_if(matchingObservations_to.begin(), matchingObservations_to.end(), [from_path](string sweep)
      {
         string to_path = getLogName(sweep);
         return to_path== from_path;
      });

      cout<<"From sweep "<<sweep_path<<endl;
      if (it != matchingObservations_to.end())
      {
         cout<<"Corresponding sweep "<<*it<<endl;
      } else {
         cout<<"No corresponding sweep"<<endl;
         continue;
      }

      // base path
      unsigned found = (sweep_path).find_last_of("/");
      std::string base_path = (sweep_path).substr(0,found+1);

      // destination path
      found = (*it).find_last_of("/");
      std::string destination_path = (*it).substr(0,found+1);
      cout<<"Destination path "<<destination_path<<endl;

      QStringList labelFiles = QDir(base_path.c_str()).entryList(QStringList("*label*.txt"));
//      QStringList objectFiles = QDir(base_path.c_str()).entryList(QStringList("*object*.jpg"));

      for (auto file : labelFiles)
      {
         string completeFile = base_path + file.toStdString();

         string destFile = destination_path + file.toStdString();
         QFile::copy(completeFile.c_str(), destFile.c_str());
         cout<<"Copied file "<<completeFile<<" --- to --- "<<destFile<<endl;

      }

//      for (auto file : objectFiles)
//      {
//         string completeFile = base_path + file.toStdString();

//         string destFile = destination_path + file.toStdString();
//         QFile::copy(completeFile.c_str(), destFile.c_str());
//         cout<<"Copied file "<<completeFile<<" --- to --- "<<destFile<<endl;

//      }

   }
}
