#include "ModelStorage/ModelStorage.h"
#include "Util/Util.h"
#include "CameraOptimizer/CameraOptimizer.h"

void train_cam_sweep(std::string path, CameraOptimizerGridXYZ * co){
    printf("path: %s\n",path.c_str());
    if ( ! boost::filesystem::exists( path ) ){return ;}

    SimpleXMLParser<pcl::PointXYZRGB> parser;
    SimpleXMLParser<pcl::PointXYZRGB>::RoomData roomData  = parser.loadRoomFromXML(path,std::vector<std::string>{"RoomIntermediateCloud","IntermediatePosition"});
    int metaroom_nrviews = roomData.vIntermediateRoomClouds.size();

    std::vector<reglib::RGBDFrame * > frames;
    std::vector<cv::Mat> rels;
    Eigen::Matrix4d m2 = Eigen::Matrix4d ::Identity();
    if(roomData.vIntermediateRoomClouds.size() != 0 ){m2 = quasimodo_brain::getMat(roomData.vIntermediateRoomCloudTransforms[0]);}

    size_t max_nr_frames = roomData.vIntermediateRoomClouds.size();
    for (size_t i=0; i < max_nr_frames; i++){
        printf("loading intermedite %i\n",i);
        image_geometry::PinholeCameraModel aCameraModel = roomData.vIntermediateRoomCloudCamParamsCorrected[i];
        reglib::Camera * cam		= new reglib::Camera();
        cam->fx = aCameraModel.fx();
        cam->fy = aCameraModel.fy();
        cam->cx = aCameraModel.cx();
        cam->cy = aCameraModel.cy();

        Eigen::Matrix4d m = m2*quasimodo_brain::getMat(roomData.vIntermediateRoomCloudTransformsRegistered[i]);
        reglib::RGBDFrame * frame = new reglib::RGBDFrame(cam,roomData.vIntermediateRGBImages[i],5.0*roomData.vIntermediateDepthImages[i],0, m,true,"");
        frames.push_back(frame);
        rels.push_back(co->getPixelReliability(frame));
    }

    for(unsigned int i = 0; i < frames.size(); i++){
        printf("%i / %i\n",i+1,frames.size());
        for(unsigned int j = 0; j < frames.size(); j++){
            if(i == j){continue;}
            Eigen::Matrix4d m = frames[j]->pose.inverse() * frames[i]->pose;
            co->addTrainingData( frames[i], frames[j], m, rels[i], rels[j]);
            co->normalize();
        }
        co->show(false);
    }

    for (size_t i=0; i < max_nr_frames; i++){
        delete frames[i];
    }
    frames.clear();
}

void train_cam_path(std::string path, CameraOptimizerGridXYZ * co){
    std::vector<std::string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<pcl::PointXYZRGB>(path);
    for (unsigned int i = 0; i < sweep_xmls.size(); i++) {
    	printf("room: %i / %i\n",i+1,sweep_xmls.size());
        train_cam_sweep(sweep_xmls[i],co);
        co->save(path+"/sweep_CameraOptimizerGridXYZ.bin");
    }
}

int main(int argc, char **argv){
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
    viewer->setBackgroundColor(1.0,1.0,1.0);
    viewer->removeAllPointClouds();
    viewer->spinOnce();

    CameraOptimizerGridXYZ * co = new CameraOptimizerGridXYZ(64,48,10,100);
    co->setVisualization(viewer,1);

    for(int i = 1; i < argc;i++){
        printf("input: %s\n",argv[i]);
        train_cam_path(std::string(argv[i]),co);
    }
    if(argc != 1){
        co->save("sweep_CameraOptimizerGridXYZ.bin");
        co->show(true);
    }
}
