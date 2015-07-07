#include <vector>
#include <boost/lexical_cast.hpp>
#include <pcl/registration/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

#include "load_utilities.h"
#include "semantic_map/room_xml_parser.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef typename Cloud::Ptr CloudPtr;

void create_int_images(std::string sweepPath)
{
   auto intCloudData = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointType>(sweepPath,false);

   unsigned found = sweepPath.find_last_of("/");
   std::string base_path = sweepPath.substr(0,found+1);

   int image_counter =0;
   for (auto image : intCloudData.vIntermediateRGBImages)
   {
      std::stringstream ss; ss<<base_path;ss<<"rgb_"<<std::setfill('0')<<std::setw(4)<<image_counter<<".jpg";
      std::stringstream ss2; ss2<<base_path;ss2<<"depth_"<<std::setfill('0')<<std::setw(4)<<image_counter<<".png";
//      std::cout<<"RGB Image would be saved at: "<<ss.str()<<std::endl;
//      std::cout<<"Depth Image would be saved at: "<<ss2.str()<<std::endl;

      cv::imwrite(ss.str(), image);
      intCloudData.vIntermediateDepthImages[image_counter]*=10;
      cv::imwrite(ss2.str(), intCloudData.vIntermediateDepthImages[image_counter]);

      image_counter++;
   }
}

void create_point_cloud_from_label(std::string label_path, std::string sweepPath)
{
   auto intCloudData = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointType>(sweepPath,false);
   SemanticRoomXMLParser<PointType> parser();
   SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(sweepPath,false);
   Eigen::Matrix4f roomTransform = aRoom.getRoomTransform();
   auto intCloudTrReg = aRoom.getIntermediateCloudTransformsRegistered();

   int index = label_path.find("rgb");
   std::string int_id = label_path.substr(index+4,4);
   int intCloudNo = boost::lexical_cast<int>(int_id);

   // find cloud transform, either registered or original   tf::StampedTransform cloudTransform;
   tf::StampedTransform cloudTransform;
   if (intCloudTrReg.size() == 0)
   {
      cloudTransform = intCloudData.vIntermediateRoomCloudTransforms[intCloudNo];
   } else {
      cloudTransform = intCloudTrReg[intCloudNo];
   }

   index = label_path.find_last_of(".");
   std::string rootPcdName = label_path.substr(0,index+1);
   rootPcdName += "pcd";

   cv::Mat label_image = cv::imread(label_path);
   CloudPtr label_cloud(new Cloud());
   CloudPtr correspIntCloud = intCloudData.vIntermediateRoomClouds[intCloudNo];

   pcl::PointXYZRGB point;
   for (size_t y = 0; y < label_image.rows; ++y) {
      for (size_t x = 0; x < label_image.cols; ++x) {

         if ((label_image.at<cv::Vec3b>(y, x)[0] == 255) &&
             (label_image.at<cv::Vec3b>(y, x)[1] == 255) &&
             (label_image.at<cv::Vec3b>(y, x)[2] == 255))
         {
            if (pcl_isfinite(correspIntCloud->points[y*label_image.cols + x].x))
            {
               label_cloud->push_back(correspIntCloud->points[y*label_image.cols + x]);
            }
         }
      }
   }

   // remove noise
   Eigen::Vector4f centroid;
   pcl::compute3DCentroid(*label_cloud, centroid);

   std::vector<pcl::PointXYZRGB> points; //points.assign(label_cloud->points.begin(), label_cloud->points.end());
   for (int i=0; i<label_cloud->points.size(); i++)
   {
       points.push_back(label_cloud->points[i]);;
   }

   bool has_infs = false;
   std::sort(label_cloud->points.begin(), label_cloud->points.end(),
             [&centroid, &has_infs](PointType const& a, PointType const& b)
   {
      Eigen::Vector4f eigen_a, eigen_b;
      eigen_a[0] = a.x;eigen_a[1] = a.y;eigen_a[2] = a.z;eigen_a[3] = 0.0f;
      eigen_b[0] = b.x;eigen_b[1] = b.y;eigen_b[2] = b.z;eigen_b[3] = 0.0f;
      double dist_a = pcl::distances::l2(eigen_a,centroid);
      double dist_b = pcl::distances::l2(eigen_b,centroid);
      if (std::isnan(dist_a) || std::isnan(dist_b) || std::isinf(dist_a) || std::isinf(dist_b))
      {
          std::cout<<"Nan or something fishy "<<eigen_a<<" "<<eigen_b<<" "<<centroid<<std::endl;
      }
      return dist_a <= dist_b;
   });

   CloudPtr filteredCloud(new Cloud());
   for (int i=0; i<0.95 * label_cloud->points.size();i++)
   {
      filteredCloud->push_back(label_cloud->points[i]);
   }

   //   // transform into the room frame of reference
   CloudPtr transformedCloud(new Cloud());
   pcl_ros::transformPointCloud(*filteredCloud, *transformedCloud,cloudTransform);
   pcl::transformPointCloud (*transformedCloud, *transformedCloud, roomTransform);

   pcl::io::savePCDFileBinary(rootPcdName, *transformedCloud);
   std::cout<<"Label point cloud saved at "<<rootPcdName<<std::endl;

   return;
}
