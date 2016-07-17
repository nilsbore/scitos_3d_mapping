#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <string.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"

#include "quasimodo_msgs/segment_model.h"


#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

#include "modelupdater/ModelUpdater.h"
#include "core/RGBDFrame.h"

#include "Util/Util.h"


using namespace std;


bool visualization = false;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


//Strategy:
//Register frames to background+other frames in the same model DONE
//Determine occlusions inside current model to filter out motion(most likeley people etc) //Done
    //Remove from current model
//Determine occlusion of current model to background
    //These pixels are dynamic objects
//Update background
    //How?

bool segment_model(quasimodo_msgs::segment_model::Request  & req, quasimodo_msgs::segment_model::Response & res){
	printf("segment_model\n");
	reglib::Model * bg = quasimodo_brain::getModelFromMSG(req.backgroundmodel);
	std::vector< reglib::Model * > models;
	for(unsigned int i = 0; i < req.models.size(); i++){
		models.push_back(quasimodo_brain::getModelFromMSG(req.models[i]));
	}

	vector<Eigen::Matrix4d> cp;
	vector<reglib::RGBDFrame*> cf;
	vector<reglib::ModelMask*> mm;

	vector<Eigen::Matrix4d> cp_front;
	vector<reglib::RGBDFrame*> cf_front;
	vector<reglib::ModelMask*> mm_front;
	for(int j = 0; j < models.size(); j++){
		cp_front.push_back(models.front()->relativeposes.front().inverse() * models[j]->relativeposes.front());
		cf_front.push_back(models[j]->frames.front());
		mm_front.push_back(models[j]->modelmasks.front());
	}
	printf("cp_front: %i\n",cp_front.size());

	if(models.size() > 2){
		reglib::MassRegistrationPPR2 * massreg = new reglib::MassRegistrationPPR2(0.05);
		massreg->timeout = 1200;
		massreg->viewer = viewer;
		massreg->visualizationLvl = 0;

		massreg->maskstep = 5;//std::max(1,int(0.4*double(models[i]->frames.size())));
		massreg->nomaskstep = 5;//std::max(3,int(0.5+0.*double(models[i]->frames.size())));//std::max(1,int(0.5+1.0*double(model->frames.size())));
		massreg->nomask = true;
		massreg->stopval = 0.0005;

		massreg->setData(cf_front,mm_front);


		reglib::MassFusionResults mfr_front = massreg->getTransforms(cp_front);

		for(int j = models.size()-1; j >= 0; j--){
			Eigen::Matrix4d change = mfr_front.poses[j] * cp_front[j].inverse();
			for(unsigned int k = 0; k < models[j]->relativeposes.size(); k++){
				cp.push_back(change * models.front()->relativeposes.front().inverse() * models[j]->relativeposes[k]);
				cf.push_back(models[j]->frames[k]);
				mm.push_back(models[j]->modelmasks[k]);
			}
		}
	}else{
		for(int j = models.size()-1; j >= 0; j--){
			Eigen::Matrix4d change = Eigen::Matrix4d::Identity();//mfr_front.poses[j] * cp_front[j].inverse();
			for(unsigned int k = 0; k < models[j]->relativeposes.size(); k++){
				cp.push_back(change * models.front()->relativeposes.front().inverse() * models[j]->relativeposes[k]);
				cf.push_back(models[j]->frames[k]);
				mm.push_back(models[j]->modelmasks[k]);
			}
		}
	}


	printf("cp: %i\n",cp.size());

	reglib::MassRegistrationPPR2 * massreg2 = new reglib::MassRegistrationPPR2(0.0);
	massreg2->timeout = 1200;
	massreg2->viewer = viewer;
	massreg2->visualizationLvl = 1;

	massreg2->maskstep = 10;//std::max(1,int(0.4*double(models[i]->frames.size())));
	massreg2->nomaskstep = 10;//std::max(3,int(0.5+0.*double(models[i]->frames.size())));//std::max(1,int(0.5+1.0*double(model->frames.size())));
	massreg2->nomask = true;
	massreg2->stopval = 0.0005;

	massreg2->setData(cf,mm);
	reglib::MassFusionResults mfr2 = massreg2->getTransforms(cp);
	cp = mfr2.poses;

	reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom();
	reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( models.front(), reg);
	mu->occlusion_penalty               = 15;
	mu->massreg_timeout                 = 60*4;
	mu->viewer							= viewer;

    vector<cv::Mat> masks;
    for(unsigned int i = 0; i < cf.size(); i++){
        cv::Mat mask;
        mask.create(cf[i]->camera->height,cf[i]->camera->width,CV_8UC1);
        unsigned char * maskdata = (unsigned char *)(mask.data);
        for(unsigned int i = 0; i < cf[i]->camera->height*cf[i]->camera->width;i++){maskdata[i] = 255;}
        masks.push_back(mask);
    }
    std::vector<cv::Mat> internal_masks = mu->computeDynamicObject(0,cp,cf,masks);//Determine self occlusions
    std::vector<cv::Mat> dynamic_masks = mu->computeDynamicObject(bg,cp,cf,internal_masks);//Determine occlusion of background occlusions
    //add new frames to background
    //Compute minimum required frames to capture background
    for(unsigned int i = 0; i < cf.size(); i++){

        cv::Mat mask;
        mask.create(cf[i]->camera->height,cf[i]->camera->width,CV_8UC1);
        unsigned char * maskdata = (unsigned char *)(mask.data);
        unsigned char * internalmaskdata = (unsigned char *)(internal_masks[i].data);
        unsigned char * dynamicmaskdata = (unsigned char *)(dynamic_masks[i].data);
        for(unsigned int i = 0; i < cf[i]->camera->height*cf[i]->camera->width;i++){
            maskdata[i] = std::max(internalmaskdata[i],dynamicmaskdata[i]);
        }

        bg->frames.push_back(cf[i]);
        bg->relativeposes.push_back(cp[i]);
        bg->modelmasks.push_back(new reglib::ModelMask(mask));
    }

	return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "segmentationserver");
	ros::NodeHandle n;

	int inputstate = -1;
	for(int i = 1; i < argc;i++){
		printf("input: %s\n",argv[i]);
		if(std::string(argv[i]).compare("-v") == 0){           printf("visualization turned on\n");                visualization = true;}
		/*
		if(		std::string(argv[i]).compare("-c") == 0){	printf("camera input state\n"); inputstate = 1;}
		else if(std::string(argv[i]).compare("-l") == 0){	printf("reading all models at %s (the path defined from -p)\n",savepath.c_str());
			std::vector<std::string> folderdata;
			int r = getdir(savepath+"/",folderdata);
			for(unsigned int fnr = 0; fnr < folderdata.size(); fnr++){
				printf("%s\n",folderdata[fnr].c_str());
			}
			exit(0);}
		else if(std::string(argv[i]).compare("-m") == 0){	printf("model input state\n");	inputstate = 2;}
		else if(std::string(argv[i]).compare("-p") == 0){	printf("path input state\n");	inputstate = 3;}
		else if(std::string(argv[i]).compare("-occlusion_penalty") == 0){printf("occlusion_penalty input state\n");inputstate = 4;}
		else if(std::string(argv[i]).compare("-massreg_timeout") == 0){printf("massreg_timeout input state\n");inputstate = 5;}
		else if(std::string(argv[i]).compare("-search") == 0){printf("pointcloud search input state\n");run_search = true; inputstate = 6;}
		else
		else if(std::string(argv[i]).compare("-v_init") == 0){      printf("visualization of init turned on\n");        visualization = true; inputstate = 8;}
		else if(std::string(argv[i]).compare("-v_refine") == 0 || std::string(argv[i]).compare("-v_ref") == 0){	printf("visualization refinement turned on\n");     visualization = true; inputstate = 9;}
		else if(std::string(argv[i]).compare("-v_register") == 0 || std::string(argv[i]).compare("-v_reg") == 0){	printf("visualization registration turned on\n");	visualization = true; inputstate = 10;}
		else if(std::string(argv[i]).compare("-v_scoring") == 0 || std::string(argv[i]).compare("-v_score") == 0 || std::string(argv[i]).compare("-v_sco") == 0){	printf("visualization scoring turned on\n");        visualization = true; show_scoring = true;}
		else if(std::string(argv[i]).compare("-v_db") == 0){        printf("visualization db turned on\n");             visualization = true; show_db = true;}
		else if(inputstate == 1){
			reglib::Camera * cam = reglib::Camera::load(std::string(argv[i]));
			delete cameras[0];
			cameras[0] = cam;
		}else if(inputstate == 2){
			reglib::Model * model = reglib::Model::load(cameras[0],std::string(argv[i]));
			sweepid_counter = std::max(int(model->modelmasks[0]->sweepid + 1), sweepid_counter);
			modeldatabase->add(model);
			//addToDB(modeldatabase, model,false);
			model->last_changed = ++current_model_update;
			show_sorted();
		}else if(inputstate == 3){
			savepath = std::string(argv[i]);
		}else if(inputstate == 4){
			occlusion_penalty = atof(argv[i]); printf("occlusion_penalty set to %f\n",occlusion_penalty);
		}else if(inputstate == 5){
			massreg_timeout = atof(argv[i]); printf("massreg_timeout set to %f\n",massreg_timeout);
		}else if(inputstate == 6){
			search_timeout = atof(argv[i]); printf("search_timeout set to %f\n",search_timeout);
			if(search_timeout == 0){
				run_search = false;
			}
		}else if(inputstate == 8){
			show_init_lvl = atoi(argv[i]);
		}else if(inputstate == 9){
			show_refine_lvl = atoi(argv[i]);
		}else if(inputstate == 10){
			show_reg_lvl = atoi(argv[i]);
		}
		*/
	}

	if(visualization){
		viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
		viewer->addCoordinateSystem(0.01);
		viewer->setBackgroundColor(0.0,0.0,0.0);
	}


	ros::ServiceServer service = n.advertiseService("segment_model", segment_model);
	ROS_INFO("Ready to add use segment_model.");

	ros::spin();

/*
exit(0);
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0.5, 0, 0.5);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	ros::NodeHandle pn("~");
	for(int ar = 1; ar < argc; ar++){
		string overall_folder = std::string(argv[ar]);

		vector<string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(overall_folder);
		printf("sweep_xmls\n");
		for (auto sweep_xml : sweep_xmls) {
			printf("sweep_xml: %s\n",sweep_xml.c_str());
			load2(sweep_xml);
		}
	}

	for(unsigned int i = 0; i < models.size(); i++){
		printf("%i -> %i\n",i,models[i]->frames.size());

		vector<Eigen::Matrix4d> cp;
		vector<reglib::RGBDFrame*> cf;
		vector<reglib::ModelMask*> mm;

		vector<Eigen::Matrix4d> cp_front;
		vector<reglib::RGBDFrame*> cf_front;
		vector<reglib::ModelMask*> mm_front;
		for(int j = 0; j <= i; j++){
			cp_front.push_back(models.front()->relativeposes.front().inverse() * models[j]->relativeposes.front());
			cf_front.push_back(models[j]->frames.front());
			mm_front.push_back(models[j]->modelmasks.front());
		}

		if(i > 0){
			reglib::MassRegistrationPPR2 * massreg = new reglib::MassRegistrationPPR2(0.05);
			massreg->timeout = 1200;
			massreg->viewer = viewer;
			massreg->visualizationLvl = 0;

			massreg->maskstep = 5;//std::max(1,int(0.4*double(models[i]->frames.size())));
			massreg->nomaskstep = 5;//std::max(3,int(0.5+0.*double(models[i]->frames.size())));//std::max(1,int(0.5+1.0*double(model->frames.size())));
			massreg->nomask = true;
			massreg->stopval = 0.0005;

			massreg->setData(cf_front,mm_front);


			reglib::MassFusionResults mfr_front = massreg->getTransforms(cp_front);

			for(int j = i; j >= 0; j--){
				Eigen::Matrix4d change = mfr_front.poses[j] * cp_front[j].inverse();
				for(unsigned int k = 0; k < models[j]->relativeposes.size(); k++){
					cp.push_back(change * models.front()->relativeposes.front().inverse() * models[j]->relativeposes[k]);
					cf.push_back(models[j]->frames[k]);
					mm.push_back(models[j]->modelmasks[k]);
				}
			}
		}else{
			for(int j = i; j >= 0; j--){
				Eigen::Matrix4d change = Eigen::Matrix4d::Identity();//mfr_front.poses[j] * cp_front[j].inverse();
				for(unsigned int k = 0; k < models[j]->relativeposes.size(); k++){
					cp.push_back(change * models.front()->relativeposes.front().inverse() * models[j]->relativeposes[k]);
					cf.push_back(models[j]->frames[k]);
					mm.push_back(models[j]->modelmasks[k]);
				}
			}
		}

		reglib::MassRegistrationPPR2 * massreg2 = new reglib::MassRegistrationPPR2(0.0);
		massreg2->timeout = 1200;
		massreg2->viewer = viewer;
		massreg2->visualizationLvl = 1;

		massreg2->maskstep = 10;//std::max(1,int(0.4*double(models[i]->frames.size())));
		massreg2->nomaskstep = 10;//std::max(3,int(0.5+0.*double(models[i]->frames.size())));//std::max(1,int(0.5+1.0*double(model->frames.size())));
		massreg2->nomask = true;
		massreg2->stopval = 0.0005;

		massreg2->setData(cf,mm);
		reglib::MassFusionResults mfr2 = massreg2->getTransforms(cp);
		cp = mfr2.poses;

		reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom();
		reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( models[i], reg);
		mu->occlusion_penalty               = 15;
		mu->massreg_timeout                 = 60*4;
		mu->viewer							= viewer;

		vector<cv::Mat> mats;
		mu->computeDynamicObject(cp,cf,mats);

		//delete mu;
	}


	*/
}