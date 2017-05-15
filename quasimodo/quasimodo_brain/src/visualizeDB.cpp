#include "ModelDatabase/ModelDatabase.h"
#include "ModelStorage/ModelStorage.h"
#include "Util/Util.h"

using namespace quasimodo_brain;

void getBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld, float & maxx, float & minx, float & maxy, float & miny, float & maxz, float & minz){
	maxx = cld->points.front().x;
	minx = cld->points.front().x;
	maxy = cld->points.front().y;
	miny = cld->points.front().y;
	maxz = cld->points.front().z;
	minz = cld->points.front().z;
	for(unsigned int i = 1; i < cld->points.size(); i++){
		maxx = std::max(cld->points[i].x,maxx);
		minx = std::min(cld->points[i].x,minx);

		maxy = std::max(cld->points[i].y,maxy);
		miny = std::min(cld->points[i].y,miny);

		maxz = std::max(cld->points[i].z,maxz);
		minz = std::min(cld->points[i].z,minz);
	}
}

void add(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld, float x, float y, float z){
	for(unsigned int i = 0; i < cld->points.size(); i++){
		cld->points[i].x += x;
		cld->points[i].y += y;
		cld->points[i].z += z;
	}
}

void addToViewer(pcl::PointXYZ prev, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, reglib::Model * model, std::string acc = "model", float startx = 0, float starty = 0, float startz = 0){
	model->recomputeModelPoints();
	printf("acc: %s\n",acc.c_str());
	printf("scores: %f %f %f %f\n",model->getScore(0),model->getScore(1),model->getScore(2),model->getScore(3));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld	= model->getPCLcloud(1,false);
	float maxx, minx, maxy, miny, maxz, minz;
	getBox(cld,maxx,minx,maxy,miny,maxz, minz);
	add(cld,startx-minx,starty-miny,startz-minz);
	viewer->addPointCloud<pcl::PointXYZRGB> (cld, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cld), acc);

	double x = 0;
	double y = 0;
	double z = 0;
	for(unsigned int i = 1; i < cld->points.size(); i++){
		x += cld->points[i].x;
		y += cld->points[i].y;
		z += cld->points[i].z;
	}

	pcl::PointXYZ curr;
	curr.x = x/double(cld->points.size());//+0.5*(maxx+minx);
	curr.y = y/double(cld->points.size());//+0.5*(maxy+miny);
	curr.z = z/double(cld->points.size());//+0.5*(maxz+minz);
	viewer->addLine<pcl::PointXYZ> (prev, curr,1,0,0,"line_"+acc);

	double prev_x = startx;
	for(unsigned int i = 0; i < model->submodels.size(); i++){
		addToViewer(curr,viewer,model->submodels[i],acc+"_"+std::to_string(i),prev_x,starty+maxy+0.3,0);
		prev_x += 0.3+maxx-minx;
	}
}

int main(int argc, char **argv){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("Modelserver Viewer"));
	viewer->addCoordinateSystem(0.1);
	viewer->setBackgroundColor(1.0,1.0,1.0);
//	viewer->addPointCloud<pcl::PointXYZRGB> (cld1, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cld1), "model1");
//	viewer->addPointCloud<pcl::PointXYZRGB> (cld2, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cld2), "model2");


	ModelStorageFile * storage = new ModelStorageFile();
	storage->print();
	storage->loadAllModels();
	storage->print();

	for (std::map<std::string,std::string>::iterator it=storage->keyPathMap.begin(); it!=storage->keyPathMap.end(); ++it){
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		reglib::Model * model = storage->fetch(it->first);

		if(model->getScore(3) > 1000 && model->getScore(2) > 250 && model->getScore(1) > 1000){
			pcl::PointXYZ curr;
			curr.x = 0;
			curr.y = 0;
			curr.z = 0;
			addToViewer(curr,viewer,model);
			viewer->spin();

			if(model->submodels.size() > 0){
				reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom(5);
				reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( model, reg);
				mu->occlusion_penalty               = 10;
				mu->viewer							= viewer;
				mu->show_scoring					= true;//fuse scoring show
				reg->visualizationLvl				= 0;

				std::vector<reglib::Model *> models;
				std::vector<Eigen::Matrix4d> rps;
				mu->addModelsToVector(models,rps,model,Eigen::Matrix4d::Identity());


//				//Show alignment
//				std::vector<std::vector < reglib::OcclusionScore > > ocs = mu->computeOcclusionScore(models,rps,1,mu->show_scoring);
//				std::vector<std::vector < float > > scores = mu->getScores(ocs);
//				std::vector<int> partition = mu->getPartition(scores,2,5,2);

//				for(unsigned int i = 0; i < scores.size(); i++){
//					for(unsigned int j = 0; j < scores.size(); j++){
//						if(scores[i][j] >= 0){printf(" ");}
//						printf("%5.5f ",0.00001*scores[i][j]);
//					}
//					printf("\n");
//				}
//				printf("partition "); for(unsigned int i = 0; i < partition.size(); i++){printf("%i ", partition[i]);} printf("\n");

				delete mu;
				delete reg;
			}
		}
		storage->fullHandback();
	}

	return 0;
}
