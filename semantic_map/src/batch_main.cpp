#include "string"
#include <QString>
#include "semantic_map_node.h"

using namespace std;

std::vector<std::string> allRooms;

typedef pcl::PointXYZRGB PointType;
typedef typename SemanticMapSummaryParser<PointType>::EntityStruct Entities;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;


void scanDir(QDir dir)
{
    dir.setNameFilters(QStringList("*.xml"));
    dir.setFilter(QDir::Files | QDir::NoDotAndDotDot | QDir::NoSymLinks);

//    qDebug() << "Scanning: " << dir.path();

    QStringList fileList = dir.entryList();
    for (int i=0; i<fileList.count(); i++)
    {
        if(fileList[i] == "room.xml" )
        {
            std::string file = dir.absolutePath().toStdString()+"/"+fileList[i].toStdString();
            ROS_INFO_STREAM("Found file: " <<dir.absolutePath().toStdString()<<"/"<<fileList[i].toStdString());
            allRooms.push_back(file);
        }
    }

    dir.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot | QDir::NoSymLinks);
    QStringList dirList = dir.entryList();
    for (int i=0; i<dirList.size(); ++i)
    {
        QString newPath = QString("%1/%2").arg(dir.absolutePath()).arg(dirList.at(i));
        scanDir(QDir(newPath));
    }
}

void sortAllClusters(std::string path)
{
    SemanticMapSummaryParser<PointType>  summaryParser;

    summaryParser.createSummaryXML();

    // update list of rooms & metarooms
    summaryParser.refresh();

     std::vector<Entities> allMetarooms = summaryParser.getMetaRooms();
     std::vector<Entities> allRooms = summaryParser.getRooms();
     ROS_INFO_STREAM("Found "<<allMetarooms.size()<<" metarooms.");

     QString rootDirPath(path.c_str());
     QDir rootDir(rootDirPath);
     if (!rootDir.exists())
     {
         QDir().mkdir(rootDirPath);
     }

     for (size_t i=0; i<allMetarooms.size(); i++)
     {
         QString mrPath = rootDirPath+"metaroom_"+QString::number(i)+"/";
         if (!QDir(mrPath).exists())
         {
             QDir().mkdir(mrPath);
         }

         boost::shared_ptr<MetaRoom<PointType> > metaroom = boost::shared_ptr<MetaRoom<PointType> >(new MetaRoom<PointType>(MetaRoomXMLParser<PointType>::loadMetaRoomFromXML(allMetarooms[i].roomXmlFile,false)));

         // save MR point cloud
         CloudPtr mrCloud = metaroom->getInteriorRoomCloud();
         QString mrCloudPath = mrPath+"metaroom_cloud.pcd";
         pcl::io::savePCDFile (mrCloudPath.toStdString(), *mrCloud, true);

         // find matching rooms and extract dynamic clusters
         for (size_t j=0; j<allRooms.size();j++)
         {
             double centroidDistance = pcl::distances::l2(allMetarooms[i].centroid,allRooms[j].centroid);
             if (! (centroidDistance < ROOM_CENTROID_DISTANCE) )
             {
                 continue;
             } else {
                 SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(allRooms[j].roomXmlFile,false);
                 CloudPtr roomDynClusters = aRoom.getDynamicClustersCloud();
                 if (roomDynClusters->points.size() > 0)
                 {
//                     boost::posix_time::ptime roomLogStartTime = aRoom.getRoomLogStartTime();
//                     QString clusterFolder = mrPath+boost::posix_time::to_simple_string(roomLogStartTime).c_str()+"/";
//                     if (!QDir(clusterFolder).exists())
//                     {
//                         QDir().mkdir(clusterFolder);
//                     }

//                     // create individual clusters
//                     std::vector<CloudPtr> vClusters = MetaRoom<PointType>::clusterPointCloud(roomDynClusters,0.05,75,10000);
//                     for (size_t k=0; k<vClusters.size(); k++)
//                     {
//                         QString clusterPath = clusterFolder+"dynamic_clusters_" +QString::number(k)+".pcd";
//                         pcl::io::savePCDFile (clusterPath.toStdString(), *vClusters[k], true);
//                     }
//                     QString clusterPath = mrPath+"dynamic_clusters_" +QString::number(j)+".pcd";
                     int roomId, patrolNumber;
                     QString roomXmlFile = allRooms[j].roomXmlFile.c_str();
//                     ROS_INFO_STREAM("Room xml file "<<roomXmlFile.toStdString());
                     int lastIndex = roomXmlFile.lastIndexOf("/");
                     if (lastIndex == -1)
                     {
                         return;
                     }

                     QString roomFolderPath = roomXmlFile.left(lastIndex);
                     int roomIDindex = roomFolderPath.lastIndexOf("_");
                     roomId = roomFolderPath.right(roomFolderPath.length()-roomIDindex-1).toInt();

                     int patrolFolderIndex = roomFolderPath.lastIndexOf("/");
                     QString patrolFolderPath = roomFolderPath.left(patrolFolderIndex);

                     int patrolNumberIndex = patrolFolderPath.lastIndexOf("_");
                     patrolNumber = patrolFolderPath.right(patrolFolderPath.length()-patrolNumberIndex-1).toInt();
//                     CloudMerge<PointType>::

                     QString clusterPath = mrPath+"dynamic_clusters_" +QString::number(patrolNumber)+".pcd";
                     pcl::io::savePCDFile (clusterPath.toStdString(), *roomDynClusters, true);
                 }
             }
         }

     }

}

void consistencyCheck()
{
    SemanticMapSummaryParser<PointType>  summaryParser;

    summaryParser.createSummaryXML();

    // update list of rooms & metarooms
    summaryParser.refresh();

     std::vector<Entities> allMetarooms = summaryParser.getMetaRooms();
     std::vector<Entities> allRooms = summaryParser.getRooms();
     ROS_INFO_STREAM("Found "<<allMetarooms.size()<<" metarooms.");

     std::vector<std::vector<std::vector<CloudPtr> > > allClusters;
     std::vector<std::vector<std::vector<Eigen::Vector4f> > > allCentroids;

     std::vector<std::vector<CloudPtr> > toBeAdded;
     std::vector<std::vector<Eigen::Vector4f> > toBeAddedCentroids;

     allClusters.resize(allMetarooms.size());
     allCentroids.resize(allMetarooms.size());
     toBeAdded.resize(allMetarooms.size());
     toBeAddedCentroids.resize(allMetarooms.size());
     for (size_t i=0; i<allMetarooms.size(); i++)
     {

         boost::shared_ptr<MetaRoom<PointType> > metaroom = boost::shared_ptr<MetaRoom<PointType> >(new MetaRoom<PointType>(MetaRoomXMLParser<PointType>::loadMetaRoomFromXML(allMetarooms[i].roomXmlFile,false)));

         // save MR point cloud
         CloudPtr mrCloud = metaroom->getInteriorRoomCloud();

         // find matching rooms and extract dynamic clusters
         for (size_t j=0; j<allRooms.size();j++)
         {
             double centroidDistance = pcl::distances::l2(allMetarooms[i].centroid,allRooms[j].centroid);
             if (! (centroidDistance < ROOM_CENTROID_DISTANCE) )
             {
                 continue;
             } else {
                 SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(allRooms[j].roomXmlFile,false);
                 CloudPtr roomDynClusters = aRoom.getDynamicClustersCloud();
                 if (roomDynClusters->points.size() > 0)
                 {

                     // create individual clusters
                     std::vector<CloudPtr> vClusters = MetaRoom<PointType>::clusterPointCloud(roomDynClusters,0.05,75,10000);
                     allClusters[i].push_back(vClusters);
                     std::vector<Eigen::Vector4f> centroids;
                     for (size_t k=0; k<vClusters.size();k++)
                     {
                         Eigen::Vector4f centroid;
                         pcl::compute3DCentroid(*vClusters[k], centroid);
                         centroids.push_back(centroid);
                     }
                     allCentroids[i].push_back(centroids);
                 }
             }
         }
     }

     double percentage_tolerance = 0.3; // a match must be found in this percentage of the cases for this cluster to be added back to the metaroom
     double centroid_distance = 0.2; // other clusters should be at most 10 cm away to be considered a match

     for (size_t q=0; q<allMetarooms.size();q++)
     {
         ROS_INFO_STREAM("Metaroom "<<q<<" has "<<allClusters[q].size()<<" matching clusters and "<< allCentroids[q].size()<<" centroids ");
     }

     ROS_INFO_STREAM("Matching clusters");
     for (size_t q=0; q<allMetarooms.size();q++)
     {

         if (allClusters[q].size() == 0)
         {
             continue;
         }

         boost::shared_ptr<MetaRoom<PointType> > metaroom = boost::shared_ptr<MetaRoom<PointType> >(new MetaRoom<PointType>(MetaRoomXMLParser<PointType>::loadMetaRoomFromXML(allMetarooms[q].roomXmlFile,false)));
         ROS_INFO_STREAM("Processing MR "<<q);
           for (int i=0; i<allClusters[q].size()-1; i++)
           {
               ROS_INFO_STREAM("Processing MR "<<q<<" cluster set "<<i);

               for (int k=0; k<allClusters[q][i].size(); k++)
               {
                   int matches = 0;
                   Eigen::Vector4f centroid1 = allCentroids[q][i][k];
                   for (int j=i+1; j<allClusters[q].size(); j++)
                   {
                       for (int l=0; l<allClusters[q][j].size(); l++)
                       {
                           Eigen::Vector4f centroid2 = allCentroids[q][j][l];
                           double distance = pcl::distances::l2(centroid1,centroid2);
                           if (distance < centroid_distance)
                           {
                               ROS_INFO_STREAM("Found a matching cluster. Matches so far "<<matches);
                               matches++;
                           }
                       }
                   }

                   if (matches > percentage_tolerance * allClusters[q].size())
                   {
                       // check if this hasn't been added already
                       bool addedAlready = false;
                       for (size_t p=0; p<toBeAddedCentroids[q].size(); p++)
                       {
                           Eigen::Vector4f centroid3 = toBeAddedCentroids[q][p];
                           double distance = pcl::distances::l2(centroid1,centroid3);
                           if (distance < centroid_distance)
                           {
                               addedAlready = true;
                               break;
                           }
                       }
                       if (!addedAlready)
                       {
                           ROS_INFO_STREAM("FOUND A CLUSTER. No matches "<<matches);
                           toBeAdded[q].push_back(allClusters[q][i][k]);
                           toBeAddedCentroids[q].push_back(allCentroids[q][i][k]);
                       }
                   }
               }
           }

           ROS_INFO_STREAM("Processed MR "<<q);
           CloudPtr consistencyUpdate(new Cloud());

           for (int i=0; i<toBeAdded[q].size(); i++)
           {
               // save for debugging
//               std::stringstream ss;
//               ss << "toBeAdded";
//               ss <<i;
//               ss <<".pcd";
//               pcl::io::savePCDFile (ss.str(), *toBeAdded[q][i], true);
               *consistencyUpdate+=*toBeAdded[q][i];
           }

           if (consistencyUpdate->points.size() > 0)
           {
                ROS_INFO_STREAM("Saving consistency update cloud for metaroom "<<q);
               metaroom->setConsistencyUpdateCloud(consistencyUpdate);
               MetaRoomXMLParser<PointType> mrParser;
               mrParser.saveMetaRoomAsXML(*metaroom);
           } else {
               ROS_INFO_STREAM("NO consistency update cloud for metaroom "<<q);
           }
     }

}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "batch_semantic_map");
    ros::NodeHandle n;

    ros::NodeHandle aRosNode("~");

    SemanticMapNode<pcl::PointXYZRGB> aSemanticMapNode(aRosNode);

    string file;
    string complete_file = "/home/rares/.semanticMap";
    bool found = aRosNode.getParam("input_xml",file);
    if(!found )
    {
        ROS_INFO_STREAM("Please provide input_xml argument");
        return -1;
    }
    complete_file+=file;
    complete_file+=".xml";

//    aSemanticMapNode.processRoomObservation(complete_file);

//    scanDir(QDir("/media/FA3407FA3407B8A1/strands_data/g4s_y1_1706014/G4S/")); // G4S data
//    scanDir(QDir("/home/rares/data/y1review_data/all_data"));
    scanDir(QDir("/home/rares/.semanticMap/"));
    ROS_INFO_STREAM("Matching files "<<allRooms.size());

    for (size_t i=0; i<allRooms.size(); i++)
    {
        aSemanticMapNode.processRoomObservation(allRooms[i]);
    }

    sortAllClusters("/home/rares/data/dynamic_clusters_longterm_4/");

//    consistencyCheck();
}
