#ifndef __FINE_MAPPING__H
#define __FINE_MAPPING__H

#include "scan.h"

class FineMapping {
public:

    typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::iterator CloudIterator;

    FineMapping(){}
    ~FineMapping(){}

    bool register_clouds_icp(Eigen::Matrix3f& R, Eigen::Vector3f& t,
                             scan* scan1, scan* scan2,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2);

    void compute_initial_transformation(Eigen::Matrix3f& R, Eigen::Vector3f& t, scan* scan1, scan* scan2);

    int fineMapping(std::vector<scan*> scans, std::vector<CloudPtr> allClouds);

private:
};


#endif
