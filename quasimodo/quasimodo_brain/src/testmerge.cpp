#include "ModelDatabase/ModelDatabase.h"
#include "ModelStorage/ModelStorage.h"
#include "Util/Util.h"

using namespace quasimodo_brain;

int main(int argc, char **argv){

	if(argc < 3){
		printf("too few args\n");
		return -1;
	}

	reglib::Model * model1						= reglib::Model::loadFast(std::string(argv[1]));
	reglib::Model * model2						= reglib::Model::loadFast(std::string(argv[2]));

	reglib::Model * model1H = new reglib::Model();
	model1->parrent = model1H;
	model1H->submodels.push_back(model1);
	model1H->submodels_relativeposes.push_back(Eigen::Matrix4d::Identity());
	model1H->recomputeModelPoints();

	reglib::Model * model2H = new reglib::Model();
	model2->parrent = model2H;
	model2H->submodels.push_back(model2);
	model2H->submodels_relativeposes.push_back(Eigen::Matrix4d::Identity());
	model2H->recomputeModelPoints();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld1	= model1H->getPCLcloud(1,false);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld2	= model2H->getPCLcloud(1,false);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("Modelserver Viewer"));
	viewer->addCoordinateSystem(0.1);
	viewer->setBackgroundColor(1.0,1.0,1.0);
	viewer->addPointCloud<pcl::PointXYZRGB> (cld1, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cld1), "model1");
	viewer->addPointCloud<pcl::PointXYZRGB> (cld2, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cld2), "model2");
	viewer->spin();

	reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom(5);
	reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( model2H, reg);
	mu->occlusion_penalty               = 10;
	mu->viewer							= viewer;
	mu->show_scoring					= true;//fuse scoring show
	reg->visualizationLvl				= 0;

	reglib::FusionResults fr = mu->registerModel(model1H);
	if(fr.score > 100){
		reglib::UpdatedModels ud = mu->fuseData(&fr, model2H, model1H);
		delete mu;
		delete reg;
	}


	return 0;
}
