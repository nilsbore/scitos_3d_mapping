#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vector>
#include <QDir>



#include "load_utilities.h"
#include "labeller.h"

#include "object_matches.h"
#include "dynamic_object.h"
#include "dynamic_object_xml_parser.h"



typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;


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

   pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer (argc, argv, "Object viewer");

   vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(path, waypId);
   ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size());

   DynamicObjectXMLParser parser;
   sort(matchingObservations.begin(), matchingObservations.end());

   vector<ObjectMatches::Ptr> allMatches;
   vector<vector<string>> allMatchFiles;

   for (size_t i=0; i<matchingObservations.size();i++)
   {
      ROS_INFO_STREAM("Parsing "<<matchingObservations[i]);

      // find label image files in this folder
      unsigned found = matchingObservations[i].find_last_of("/");
      std::string base_path = matchingObservations[i].substr(0,found+1);
      QStringList xmlFiles = QDir(base_path.c_str()).entryList(QStringList("*label*.xml"));

      cout<<"Found "<<xmlFiles.size()<<" labelled objects."<<endl;

      for (size_t i=0; i<xmlFiles.size(); i++)
      {
         DynamicObject::Ptr parsed = parser.loadFromXML(base_path+xmlFiles[i].toStdString());

         bool found = false;
//         for (auto match : allMatches)
         for (size_t j=0; j<allMatches.size();j++)
         {
            ObjectMatches::Ptr match = allMatches[j];
            if (match->m_Matches[0]->m_label == parsed->m_label)
            {
               match->addMatch(parsed);
               allMatchFiles[j].push_back(base_path+xmlFiles[i].toStdString());
               found = true;
               break;
            }
         }
         if (!found)
         {
            ObjectMatches::Ptr newMatch = ObjectMatches::Ptr(new ObjectMatches);
            vector<string> newMatchFile = {base_path+xmlFiles[i].toStdString()};
            allMatchFiles.push_back(newMatchFile);
            newMatch->addMatch(parsed);
            allMatches.push_back(newMatch);
         }

      }

   }

   int counter =0;
   for (auto matches: allMatches)
   {
      cout<<"Class label "<<matches->m_Matches[0]->m_label<<"  instances  "<<matches->m_Matches.size()<<endl;
      CloudPtr combinedClusterCloud( new Cloud());
      for (auto object: matches->m_Matches)
      {
         *combinedClusterCloud += *object->m_points;
      }

      for (string file : allMatchFiles[counter])
      {
         cout<<file<<endl;
      }
      counter++;


//      pcl::visualization::PointCloudColorHandlerCustom<PointType> cluster_handler (combinedClusterCloud, 0, 255, 0);
      p->addPointCloud (combinedClusterCloud, "cluster");
      p->spin();
      p->removeAllPointClouds();

   }


}
