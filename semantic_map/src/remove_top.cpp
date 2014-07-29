#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>

typedef pcl::PointXYZRGB PointType;

using namespace std;

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        cout<<"Please provide file in 1, file in 2 and file out as arguments."<<argc<<endl;
        exit(-1);
    }

    pcl::PCDReader reader;

    pcl::PointCloud<PointType>::Ptr cloud1 (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud2 (new pcl::PointCloud<PointType>);
    reader.read (argv[1], *cloud1);
    cout<<"Input file 1 has "<<cloud1->points.size()<<" points."<<endl;
    reader.read (argv[2], *cloud2);
    cout<<"Input file 2 has "<<cloud2->points.size()<<" points."<<endl;


    pcl::PointCloud<PointType>::Ptr sum (new pcl::PointCloud<PointType>);
    *sum+=*cloud1;
    *sum+=*cloud2;

    cout<<"Sum file has "<<sum->points.size()<<" points."<<endl;

    pcl::io::savePCDFile (argv[3], *sum, true);


}
