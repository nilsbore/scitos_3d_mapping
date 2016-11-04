#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <string.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"

#include "metaroom_xml_parser/simple_xml_parser.h"
#include "metaroom_xml_parser/simple_summary_parser.h"

#include <tf_conversions/tf_eigen.h>

#include "ros/ros.h"
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include <tf_conversions/tf_eigen.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"
#include "quasimodo_msgs/segment_model.h"
#include "quasimodo_msgs/metaroom_pair.h"

#include "ros/ros.h"
#include <quasimodo_msgs/query_cloud.h>
#include <quasimodo_msgs/visualize_query.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"

#include "metaroom_xml_parser/simple_xml_parser.h"
#include "metaroom_xml_parser/load_utilities.h"

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

#include "modelupdater/ModelUpdater.h"
#include "core/RGBDFrame.h"
#include "Util/Util.h"

// Services
#include <object_manager_msgs/DynamicObjectsService.h>
#include <object_manager_msgs/GetDynamicObjectService.h>
#include <object_manager_msgs/ProcessDynamicObjectService.h>

// Registration service
#include <observation_registration_services/ObjectAdditionalViewRegistrationService.h>

// Additional view mask service
#include <object_manager_msgs/DynamicObjectComputeMaskService.h>


#include <sys/time.h>
#include <sys/resource.h>

// Custom messages
#include "object_manager/dynamic_object.h"
#include "object_manager/dynamic_object_xml_parser.h"
#include "object_manager/dynamic_object_utilities.h"
#include "object_manager/dynamic_object_mongodb_interface.h"

#include <object_manager_msgs/DynamicObjectTracks.h>
#include <object_manager_msgs/DynamicObjectTrackingData.h>


#include <semantic_map_msgs/RoomObservation.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>


std::vector< ros::ServiceServer > m_DynamicObjectsServiceServers;
std::vector< ros::ServiceServer > m_GetDynamicObjectServiceServers;

typedef typename object_manager_msgs::DynamicObjectsService::Request DynamicObjectsServiceRequest;
typedef typename object_manager_msgs::DynamicObjectsService::Response DynamicObjectsServiceResponse;

typedef typename object_manager_msgs::GetDynamicObjectService::Request GetDynamicObjectServiceRequest;
typedef typename object_manager_msgs::GetDynamicObjectService::Response GetDynamicObjectServiceResponse;

typedef typename object_manager_msgs::ProcessDynamicObjectService::Request ProcessDynamicObjectServiceRequest;
typedef typename object_manager_msgs::ProcessDynamicObjectService::Response ProcessDynamicObjectServiceResponse;

ros::ServiceClient segmentation_client;
using namespace std;
using namespace semantic_map_load_utilties;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> Tree;
typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;

using namespace std;
using namespace semantic_map_load_utilties;

std::string overall_folder = "";
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
int visualization_lvl			= 0;
int visualization_lvl_regref	= 0;
int visualization_lvl_regini	= 0;
std::string outtopic			= "";
std::string modelouttopic		= "";
ros::Publisher out_pub;
ros::Publisher model_pub;

std::vector< ros::Publisher > m_PublisherStatuss;
std::vector< ros::Publisher > out_pubs;
std::vector< ros::Publisher > model_pubs;

std::string saveVisuals = "";

std::string posepath = "testposes.xml";
std::vector<Eigen::Matrix4d> sweepPoses;
reglib::Camera * basecam;

bool recomputeRelativePoses = false;


bool testDynamicObjectServiceCallback(std::string path);
bool dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res);

void signal_callback_handler(int signum){
	printf("Caught signal %d\n",signum);
	exit(signum);
}

void remove_old_seg(std::string sweep_folder){
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (sweep_folder.c_str())) != NULL) {
		/* print all the files and directories within directory */
		while ((ent = readdir (dir)) != NULL) {
			std::string file = std::string(ent->d_name);

			if (file.find("dynamic_obj") !=std::string::npos && (file.find(".xml") !=std::string::npos || file.find(".pcd") !=std::string::npos)){
				printf ("removing %s\n", ent->d_name);
				std::remove((sweep_folder+"/"+file).c_str());
			}

			if (file.find("dynamicmask") !=std::string::npos && file.find(".png") !=std::string::npos){
				printf ("removing %s\n", ent->d_name);
				std::remove((sweep_folder+"/"+file).c_str());
			}

			if (file.find("moving_obj") !=std::string::npos && (file.find(".xml") !=std::string::npos || file.find(".pcd") !=std::string::npos)){
				printf ("removing %s\n", ent->d_name);
				std::remove((sweep_folder+"/"+file).c_str());
			}

			if (file.find("movingmask") !=std::string::npos && file.find(".png") !=std::string::npos){
				printf ("removing %s\n", ent->d_name);
				std::remove((sweep_folder+"/"+file).c_str());
			}

			//			if (file.find("_object_") !=std::string::npos && file.find(".xml") !=std::string::npos){
			//				printf ("removing %s\n", ent->d_name);
			//				std::remove((sweep_folder+"/"+file).c_str());
			//			}
			//			if (file.find("_object_") !=std::string::npos && file.find(".pcd") !=std::string::npos){
			//				printf ("maybe: removing %s\n", ent->d_name);
			//				printf("%i\n",file.find("_additional_view_") == std::string::npos);
			//				//file.find("_additional_view_") == std::string::npos;
			//				//std::remove((sweep_folder+"/"+file).c_str());
			//			}
		}
		closedir (dir);
	}
}

void writePose(QXmlStreamWriter* xmlWriter, Eigen::Matrix4d pose){
	Eigen::Quaterniond q = Eigen::Quaterniond ((Eigen::Affine3d (pose)).rotation());

	xmlWriter->writeStartElement("Translation");
	xmlWriter->writeStartElement("x");
	xmlWriter->writeCharacters(QString::number(pose(0,3)));
	xmlWriter->writeEndElement();
	xmlWriter->writeStartElement("y");
	xmlWriter->writeCharacters(QString::number(pose(1,3)));
	xmlWriter->writeEndElement();
	xmlWriter->writeStartElement("z");
	xmlWriter->writeCharacters(QString::number(pose(2,3)));
	xmlWriter->writeEndElement();
	xmlWriter->writeEndElement(); // Translation

	xmlWriter->writeStartElement("Rotation");
	xmlWriter->writeStartElement("w");
	xmlWriter->writeCharacters(QString::number(q.w()));
	xmlWriter->writeEndElement();
	xmlWriter->writeStartElement("x");
	xmlWriter->writeCharacters(QString::number(q.x()));
	xmlWriter->writeEndElement();
	xmlWriter->writeStartElement("y");
	xmlWriter->writeCharacters(QString::number(q.y()));
	xmlWriter->writeEndElement();
	xmlWriter->writeStartElement("z");
	xmlWriter->writeCharacters(QString::number(q.z()));
	xmlWriter->writeEndElement();
	xmlWriter->writeEndElement(); //Rotation
}

void writeXml(std::string xmlFile, std::vector<reglib::RGBDFrame *> & frames, std::vector<Eigen::Matrix4d> & poses){
	int slash_pos = xmlFile.find_last_of("/");
	std::string sweep_folder = xmlFile.substr(0, slash_pos) + "/";

	QFile file(xmlFile.c_str());
	if (file.exists()){file.remove();}


	if (!file.open(QIODevice::ReadWrite | QIODevice::Text))
	{
		std::cerr<<"Could not open file "<<sweep_folder<<"additionalViews.xml to save views as XML"<<std::endl;
		return;
	}

	QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
	xmlWriter->setDevice(&file);

	xmlWriter->writeStartDocument();
	xmlWriter->writeStartElement("Views");
	xmlWriter->writeAttribute("number_of_views", QString::number(frames.size()));
	for(unsigned int i = 0; i < frames.size(); i++){
		reglib::RGBDFrame * frame = frames[i];
		char buf [1024];

		xmlWriter->writeStartElement("View");
		sprintf(buf,"%s/view_RGB%10.10i.png",sweep_folder.c_str(),i);
		cv::imwrite(buf, frame->rgb );

		sprintf(buf,"view_RGB%10.10i.png",i);
		xmlWriter->writeAttribute("RGB", QString(buf));


		sprintf(buf,"%s/view_DEPTH%10.10i.png",sweep_folder.c_str(),i);
		cv::imwrite(buf, frame->depth );
		sprintf(buf,"view_DEPTH%10.10i.png",i);
		xmlWriter->writeAttribute("DEPTH", QString(buf));

		long nsec = 1e9*((frame->capturetime)-std::floor(frame->capturetime));
		long sec = frame->capturetime;
		xmlWriter->writeStartElement("Stamp");

		xmlWriter->writeStartElement("sec");
		xmlWriter->writeCharacters(QString::number(sec));
		xmlWriter->writeEndElement();

		xmlWriter->writeStartElement("nsec");
		xmlWriter->writeCharacters(QString::number(nsec));
		xmlWriter->writeEndElement();

		xmlWriter->writeEndElement(); // Stamp

		xmlWriter->writeStartElement("Camera");

		xmlWriter->writeStartElement("fx");
		xmlWriter->writeCharacters(QString::number(frame->camera->fx));
		xmlWriter->writeEndElement();

		xmlWriter->writeStartElement("fy");
		xmlWriter->writeCharacters(QString::number(frame->camera->fy));
		xmlWriter->writeEndElement();

		xmlWriter->writeStartElement("cx");
		xmlWriter->writeCharacters(QString::number(frame->camera->cx));
		xmlWriter->writeEndElement();

		xmlWriter->writeStartElement("cy");
		xmlWriter->writeCharacters(QString::number(frame->camera->cy));
		xmlWriter->writeEndElement();

		xmlWriter->writeEndElement(); // camera

		xmlWriter->writeStartElement("RegisteredPose");
		writePose(xmlWriter,poses[i]);
		xmlWriter->writeEndElement();

		xmlWriter->writeStartElement("Pose");
		writePose(xmlWriter,frame->pose);
		xmlWriter->writeEndElement();

		xmlWriter->writeEndElement();
	}
	xmlWriter->writeEndElement(); // Semantic Room
	xmlWriter->writeEndDocument();
	delete xmlWriter;
}

Eigen::Matrix4d getPose(QXmlStreamReader * xmlReader){
	QXmlStreamReader::TokenType token = xmlReader->readNext();//Translation
	QString elementName = xmlReader->name().toString();

	token = xmlReader->readNext();//fx
	double tx = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//fy
	double ty = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//cx
	double tz = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//Translation
	elementName = xmlReader->name().toString();

	token = xmlReader->readNext();//Rotation
	elementName = xmlReader->name().toString();

	token = xmlReader->readNext();//qw
	double qw = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//qx
	double qx = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//qy
	double qy = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//qz
	double qz = atof(xmlReader->readElementText().toStdString().c_str());

	token = xmlReader->readNext();//Rotation
	elementName = xmlReader->name().toString();

	Eigen::Matrix4d regpose = (Eigen::Affine3d(Eigen::Quaterniond(qw,qx,qy,qz))).matrix();
	regpose(0,3) = tx;
	regpose(1,3) = ty;
	regpose(2,3) = tz;

	return regpose;
}

int readNumberOfViews(std::string xmlFile){
	QFile file(xmlFile.c_str());
	if (!file.exists()){return -1;}

	file.open(QIODevice::ReadOnly);
	QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);


	int count = 0;
	while (!xmlReader->atEnd() && !xmlReader->hasError()){
		QXmlStreamReader::TokenType token = xmlReader->readNext();
		if (token == QXmlStreamReader::StartDocument)
			continue;

		if (xmlReader->hasError()){return -1;}

		if (token == QXmlStreamReader::StartElement){
			if (xmlReader->name() == "View"){
				count++;
			}
		}
	}
	delete xmlReader;
	return count;
}

void readViewXML(std::string xmlFile, std::vector<reglib::RGBDFrame *> & frames, std::vector<Eigen::Matrix4d> & poses, bool compute_edges = true, std::string savePath = ""){

	QFile file(xmlFile.c_str());

	if (!file.exists())
	{
		ROS_ERROR("Could not open file %s to load room.",xmlFile.c_str());
		return;
	}

	QString xmlFileQS(xmlFile.c_str());
	int index = xmlFileQS.lastIndexOf('/');
	std::string roomFolder = xmlFileQS.left(index).toStdString();


	file.open(QIODevice::ReadOnly);
	ROS_INFO_STREAM("Parsing xml file: "<<xmlFile.c_str());

	QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);


	while (!xmlReader->atEnd() && !xmlReader->hasError())
	{
		QXmlStreamReader::TokenType token = xmlReader->readNext();
		if (token == QXmlStreamReader::StartDocument)
			continue;

		if (xmlReader->hasError())
		{
			ROS_ERROR("XML error: %s",xmlReader->errorString().toStdString().c_str());
			return;
		}

		QString elementName = xmlReader->name().toString();

		if (token == QXmlStreamReader::StartElement)
		{

			if (xmlReader->name() == "View")
			{
				cv::Mat rgb;
				cv::Mat depth;
				//printf("elementName: %s\n",elementName.toStdString().c_str());
				QXmlStreamAttributes attributes = xmlReader->attributes();
				if (attributes.hasAttribute("RGB"))
				{
					std::string imgpath = attributes.value("RGB").toString().toStdString();
					printf("rgb filename: %s\n",(roomFolder+"/"+imgpath).c_str());
					rgb = cv::imread(roomFolder+"/"+imgpath, CV_LOAD_IMAGE_UNCHANGED);

					//QString rgbpath = attributes.value("RGB").toString();
					//rgb = cv::imread(roomFolder+"/"+(rgbpath.toStdString().c_str()), CV_LOAD_IMAGE_UNCHANGED);
					//rgb = cv::imread((rgbpath.toStdString()).c_str(), CV_LOAD_IMAGE_UNCHANGED);
				}else{break;}


				if (attributes.hasAttribute("DEPTH"))
				{
					std::string imgpath = attributes.value("DEPTH").toString().toStdString();
					printf("depth filename: %s\n",(roomFolder+"/"+imgpath).c_str());
					depth = cv::imread(roomFolder+"/"+imgpath, CV_LOAD_IMAGE_UNCHANGED);
					//QString depthpath = attributes.value("DEPTH").toString();
					//printf("depth filename: %s\n",depthpath.toStdString().c_str());
					//depth = cv::imread((roomFolder+"/"+depthpath.toStdString()).c_str(), CV_LOAD_IMAGE_UNCHANGED);
					//depth = cv::imread(roomFolder+"/"+(depthpath.toStdString().c_str()), CV_LOAD_IMAGE_UNCHANGED);
				}else{break;}


				token = xmlReader->readNext();//Stamp
				elementName = xmlReader->name().toString();

				token = xmlReader->readNext();//sec
				elementName = xmlReader->name().toString();
				int sec = atoi(xmlReader->readElementText().toStdString().c_str());

				token = xmlReader->readNext();//nsec
				elementName = xmlReader->name().toString();
				int nsec = atoi(xmlReader->readElementText().toStdString().c_str());
				token = xmlReader->readNext();//end stamp

				token = xmlReader->readNext();//Camera
				elementName = xmlReader->name().toString();

				reglib::Camera * cam = new reglib::Camera();

				token = xmlReader->readNext();//fx
				cam->fx = atof(xmlReader->readElementText().toStdString().c_str());

				token = xmlReader->readNext();//fy
				cam->fy = atof(xmlReader->readElementText().toStdString().c_str());

				token = xmlReader->readNext();//cx
				cam->cx = atof(xmlReader->readElementText().toStdString().c_str());

				token = xmlReader->readNext();//cy
				cam->cy = atof(xmlReader->readElementText().toStdString().c_str());

				token = xmlReader->readNext();//Camera
				elementName = xmlReader->name().toString();

				double time = double(sec)+double(nsec)/double(1e9);

				token = xmlReader->readNext();//RegisteredPose
				elementName = xmlReader->name().toString();

				Eigen::Matrix4d regpose = getPose(xmlReader);

				token = xmlReader->readNext();//RegisteredPose
				elementName = xmlReader->name().toString();


				token = xmlReader->readNext();//Pose
				elementName = xmlReader->name().toString();

				Eigen::Matrix4d pose = getPose(xmlReader);

				token = xmlReader->readNext();//Pose
				elementName = xmlReader->name().toString();

                reglib::RGBDFrame * frame = new reglib::RGBDFrame(cam,rgb,depth, time, regpose,true,savePath);
				frames.push_back(frame);
				poses.push_back(pose);
			}
		}
	}
	delete xmlReader;
}


void setBaseSweep(std::string path){
	printf("setBaseSweep(%s)\n",path.c_str());
	SimpleXMLParser<pcl::PointXYZRGB> parser;
	SimpleXMLParser<pcl::PointXYZRGB>::RoomData roomData  = parser.loadRoomFromXML(path);//, std::vector<std::string>(),false,false);
	printf("loaded room data\n");
	image_geometry::PinholeCameraModel baseCameraModel = roomData.vIntermediateRoomCloudCamParamsCorrected.front();
	if(basecam != 0){delete basecam;}
	basecam		= new reglib::Camera();
	basecam->fx = baseCameraModel.fx();
	basecam->fy = baseCameraModel.fy();
	basecam->cx = baseCameraModel.cx();
	basecam->cy = baseCameraModel.cy();

	sweepPoses.clear();
	for (size_t i=0; i<roomData.vIntermediateRoomCloudTransformsRegistered.size(); i++){
		sweepPoses.push_back(quasimodo_brain::getMat(roomData.vIntermediateRoomCloudTransformsRegistered[i]));
		std::cout << "sweepPoses" << i << std::endl;
		std::cout << sweepPoses.back() << std::endl;
	}
}

reglib::Model * processAV(std::string path, bool compute_edges = true, std::string savePath = ""){
	printf("processAV: %s\n",path.c_str());

	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	std::vector<cv::Mat> viewrgbs;
	std::vector<cv::Mat> viewdepths;
	std::vector<tf::StampedTransform > viewtfs;

	QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("*object*.xml"));
	for (auto objectFile : objectFiles){
		auto object = loadDynamicObjectFromSingleSweep<PointType>(sweep_folder+objectFile.toStdString(),false);
		for (unsigned int i=0; i<object.vAdditionalViews.size(); i++){
			CloudPtr cloud = object.vAdditionalViews[i];

			cv::Mat rgb;
			rgb.create(cloud->height,cloud->width,CV_8UC3);
			unsigned char * rgbdata = (unsigned char *)rgb.data;

			cv::Mat depth;
			depth.create(cloud->height,cloud->width,CV_16UC1);
			unsigned short * depthdata = (unsigned short *)depth.data;

			unsigned int nr_data = cloud->height * cloud->width;
			for(unsigned int j = 0; j < nr_data; j++){
				PointType p = cloud->points[j];
				rgbdata[3*j+0]	= p.b;
				rgbdata[3*j+1]	= p.g;
				rgbdata[3*j+2]	= p.r;
				depthdata[j]	= short(5000.0 * p.z);
			}

			viewrgbs.push_back(rgb);
			viewdepths.push_back(depth);
			viewtfs.push_back(object.vAdditionalViewsTransforms[i]);
		}
	}

//	{
//		reglib::Model * sweep = quasimodo_brain::load_metaroom_model(path,savePath);
//		std::vector<reglib::RGBDFrame *> frames;
//		std::vector<reglib::ModelMask *> masks;
//		std::vector<Eigen::Matrix4d> unrefined;

//		std::vector<Eigen::Matrix4d> both_unrefined;
//		//both_unrefined.push_back(Eigen::Matrix4d::Identity());

//		std::vector<double> times;
//		for(unsigned int i = 0; i < 3000 &&  i < viewrgbs.size(); i++){
//			printf("additional view: %i\n",i);
//			geometry_msgs::TransformStamped msg;
//			tf::transformStampedTFToMsg(viewtfs[i], msg);
//			long sec = msg.header.stamp.sec;
//			long nsec = msg.header.stamp.nsec;
//			double time = double(sec)+1e-9*double(nsec);

//			Eigen::Matrix4d m = quasimodo_brain::getMat(viewtfs[i]);

//			unrefined.push_back(m);
//			times.push_back(time);

//			cv::Mat fullmask;
//			fullmask.create(480,640,CV_8UC1);
//			unsigned char * maskdata = (unsigned char *)fullmask.data;
//			for(int j = 0; j < 480*640; j++){maskdata[j] = 255;}
//			masks.push_back(new reglib::ModelMask(fullmask));

//			reglib::Camera * cam		= sweep->frames.front()->camera->clone();
//			reglib::RGBDFrame * frame	= new reglib::RGBDFrame(cam,viewrgbs[i],viewdepths[i],time, m,true,savePath);//a.matrix());
//			frames.push_back(frame);

//			both_unrefined.push_back(quasimodo_brain::getMat(viewtfs[0]).inverse()*m);
//		}

//		if(frames.size() > 0){
//			reglib::Model * fullmodel = new reglib::Model();
//			fullmodel->points.clear();
//			reglib::MassRegistrationPPR2 * bgmassreg = new reglib::MassRegistrationPPR2(0.25);
//			if(savePath.size() != 0){
//				bgmassreg->savePath = savePath+"/processAV_"+std::to_string(fullmodel->id);
//			}

//			bgmassreg->timeout = 300;
//			bgmassreg->viewer = viewer;
//			bgmassreg->use_surface = true;
//			bgmassreg->use_depthedge = false;
//			bgmassreg->visualizationLvl = visualization_lvl_regref;

//			bgmassreg->maskstep = 2;
//			bgmassreg->nomaskstep = 2;

//			bgmassreg->nomask = true;
//			bgmassreg->stopval = 0.0005;
//			//bgmassreg->addModel(sweep);
//			bgmassreg->setData(frames,masks);

//			reglib::MassFusionResults bgmfr = bgmassreg->getTransforms(both_unrefined);
//			delete bgmassreg;

//			for(unsigned int i = 0; i < frames.size(); i++){
//				fullmodel->frames.push_back(frames[i]);
//				fullmodel->modelmasks.push_back(masks[i]);
//				fullmodel->relativeposes.push_back(bgmfr.poses[i]);
//			}
//			fullmodel->recomputeModelPoints();

//			if(savePath.size() != 0){
//				pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cld = fullmodel->getPCLnormalcloud(1, false);
//				pcl::io::savePCDFileBinaryCompressed (savePath+"/processAV_"+std::to_string(fullmodel->id)+"_fused.pcd", *cld);
//			}
//		}
//	}

    reglib::Model * sweep = quasimodo_brain::load_metaroom_model(path,savePath);
	for(unsigned int i = 0; (i < sweep->frames.size()) && (sweep->frames.size() == sweepPoses.size()) ; i++){
		sweep->frames[i]->pose	= sweep->frames.front()->pose * sweepPoses[i];
		sweep->relativeposes[i] = sweepPoses[i];
		if(basecam != 0){
			delete sweep->frames[i]->camera;
			sweep->frames[i]->camera = basecam->clone();
		}
	}

	std::vector<reglib::RGBDFrame *> frames;
	std::vector<reglib::ModelMask *> masks;
	std::vector<Eigen::Matrix4d> unrefined;

	std::vector<Eigen::Matrix4d> both_unrefined;
	both_unrefined.push_back(Eigen::Matrix4d::Identity());
	std::vector<double> times;
	for(unsigned int i = 0; i < 3000 &&  i < viewrgbs.size(); i++){
		printf("additional view: %i\n",i);
		geometry_msgs::TransformStamped msg;
		tf::transformStampedTFToMsg(viewtfs[i], msg);
		long sec = msg.header.stamp.sec;
		long nsec = msg.header.stamp.nsec;
		double time = double(sec)+1e-9*double(nsec);

		Eigen::Matrix4d m = quasimodo_brain::getMat(viewtfs[i]);

		cout << m << endl << endl;

		unrefined.push_back(m);
		times.push_back(time);

		cv::Mat fullmask;
		fullmask.create(480,640,CV_8UC1);
		unsigned char * maskdata = (unsigned char *)fullmask.data;
		for(int j = 0; j < 480*640; j++){maskdata[j] = 255;}
		masks.push_back(new reglib::ModelMask(fullmask));

		reglib::Camera * cam		= sweep->frames.front()->camera->clone();
        reglib::RGBDFrame * frame	= new reglib::RGBDFrame(cam,viewrgbs[i],viewdepths[i],time, m,true,savePath);//a.matrix());
		frames.push_back(frame);

		both_unrefined.push_back(sweep->frames.front()->pose.inverse()*m);
	}

	reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom();
	reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( sweep, reg);
	mu->occlusion_penalty               = 15;
	mu->massreg_timeout                 = 60*4;
	mu->viewer							= viewer;

	sweep->recomputeModelPoints();

	reglib::Model * fullmodel = new reglib::Model();
	fullmodel->savePath = savePath+"/";
	fullmodel->frames = sweep->frames;
	fullmodel->relativeposes = sweep->relativeposes;
	fullmodel->modelmasks = sweep->modelmasks;

	if(frames.size() > 0){

		reglib::RegistrationRefinement * refinement = new reglib::RegistrationRefinement();
		refinement->viewer	= viewer;
		refinement->visualizationLvl	= visualization_lvl_regini;
		refinement->normalize_matchweights = false;
		refinement->target_points = 4000;
////double register_setup_start = getTime();

		reglib::CloudData * cd1 = sweep->getCD(sweep->points.size());
		refinement->setDst(cd1);

		fullmodel->points = frames.front()->getSuperPoints(Eigen::Matrix4d::Identity(),1,false);
		reglib::CloudData * cd2	= fullmodel->getCD(fullmodel->points.size());
		refinement->setSrc(cd2);
//////printf("register_setup_start:          %5.5f\n",getTime()-register_setup_start);
////double register_compute_start = getTime();
		Eigen::Matrix4d guess = both_unrefined[1]*both_unrefined[0].inverse();

		reglib::FusionResults fr = refinement->getTransform(guess);
		Eigen::Matrix4d offset = fr.guess*guess.inverse();
		for(unsigned int i = 1; i < both_unrefined.size(); i++){
			both_unrefined[i] = offset*both_unrefined[i];
		}

		delete refinement;

		fullmodel->points.clear();
		delete cd1;
		delete cd2;

		reglib::MassRegistrationPPR2 * bgmassreg = new reglib::MassRegistrationPPR2(0.25);
		if(savePath.size() != 0){
			bgmassreg->savePath = savePath+"/processAV_"+std::to_string(fullmodel->id);
		}

		bgmassreg->timeout = 300;
		bgmassreg->viewer = viewer;
		bgmassreg->use_surface = true;
		bgmassreg->use_depthedge = false;
		bgmassreg->visualizationLvl = visualization_lvl_regref;
//		bgmassreg->maskstep = 5;
//		bgmassreg->nomaskstep = 5;

		bgmassreg->maskstep = 5;
		bgmassreg->nomaskstep = 5;

		bgmassreg->nomask = true;
		bgmassreg->stopval = 0.0005;
		bgmassreg->addModel(sweep);
		bgmassreg->setData(frames,masks);

		reglib::MassFusionResults bgmfr = bgmassreg->getTransforms(both_unrefined);

		//printf("%s\n",bgmassreg->savePath.c_str());
		//exit(0);

		delete bgmassreg;

		//		reglib::MassRegistrationPPR2 * bgmassreg2 = new reglib::MassRegistrationPPR2(0.01);

		//		bgmassreg2->timeout = 300;
		//		bgmassreg2->viewer = viewer;
		//		bgmassreg2->use_surface = true;
		//		bgmassreg2->use_depthedge = false;
		//		bgmassreg2->visualizationLvl = visualization_lvl;
		//		bgmassreg2->maskstep = 2;
		//		bgmassreg2->nomaskstep = 2;
		//		bgmassreg2->nomask = true;
		//		bgmassreg2->stopval = 0.0005;
		//		bgmassreg2->addModel(sweep);
		//		bgmassreg2->setData(frames,masks);

		//		reglib::MassFusionResults bgmfr2 = bgmassreg2->getTransforms(bgmfr.poses);
		//		delete bgmassreg2;

		for(unsigned int i = 0; i < frames.size(); i++){frames[i]->pose = sweep->frames.front()->pose * bgmfr.poses[i+1];}

		for(unsigned int i = 0; i < frames.size(); i++){
			fullmodel->frames.push_back(frames[i]);
			fullmodel->modelmasks.push_back(masks[i]);
			fullmodel->relativeposes.push_back(bgmfr.poses[i+1]);
		}
		fullmodel->recomputeModelPoints();

		if(savePath.size() != 0){
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cld = fullmodel->getPCLnormalcloud(1, false);
			pcl::io::savePCDFileBinaryCompressed (savePath+"/processAV_"+std::to_string(fullmodel->id)+"_fused.pcd", *cld);
		}
		//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Model::getPCLnormalcloud(int step, bool color){

	}else{
		fullmodel->points = sweep->points;
	}

	delete reg;
	delete mu;
	delete sweep;
	//delete basecam;

	return fullmodel;
}

void savePoses(std::string xmlFile, std::vector<Eigen::Matrix4d> poses, int maxposes = -1){
	QFile file(xmlFile.c_str());
	if (file.exists()){file.remove();}

	if (!file.open(QIODevice::ReadWrite | QIODevice::Text)){
		std::cerr<<"Could not open file "<< xmlFile <<" to save views as XML"<<std::endl;
		return;
	}

	QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
	xmlWriter->setDevice(&file);

	xmlWriter->writeStartDocument();
	xmlWriter->writeStartElement("Poses");
	//	xmlWriter->writeAttribute("number_of_poses", QString::number(poses.size()));
	for(unsigned int i = 0; i < poses.size() && (maxposes == -1 || int(i) < maxposes); i++){
		printf("saving %i\n",i);
		xmlWriter->writeStartElement("Pose");
		writePose(xmlWriter,poses[i]);
		xmlWriter->writeEndElement();
	}
	xmlWriter->writeEndElement(); // Poses
	xmlWriter->writeEndDocument();
	delete xmlWriter;
}

std::vector<Eigen::Matrix4d> readPoseXML(std::string xmlFile){
	std::vector<Eigen::Matrix4d> poses;
	QFile file(xmlFile.c_str());

	if (!file.exists()){
		ROS_ERROR("Could not open file %s to load poses.",xmlFile.c_str());
		return poses;
	}

	file.open(QIODevice::ReadOnly);
	ROS_INFO_STREAM("Parsing xml file: "<<xmlFile.c_str());

	QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);

	while (!xmlReader->atEnd() && !xmlReader->hasError()){
		QXmlStreamReader::TokenType token = xmlReader->readNext();
		if (token == QXmlStreamReader::StartDocument)
			continue;

		if (xmlReader->hasError())
		{
			ROS_ERROR("XML error: %s",xmlReader->errorString().toStdString().c_str());
			return poses;
		}

		QString elementName = xmlReader->name().toString();

		if (token == QXmlStreamReader::StartElement){
			if (xmlReader->name() == "Pose"){
				Eigen::Matrix4d pose = getPose(xmlReader);

				token = xmlReader->readNext();//Pose
				elementName = xmlReader->name().toString();

				std::cout << pose << std::endl << std::endl;
				poses.push_back(pose);
			}
		}
	}
	delete xmlReader;
	printf("done readPoseXML\n");
	return poses;
}

reglib::Model * getAVMetaroom(std::string path, bool compute_edges = true, std::string saveVisuals_sp = ""){
	printf("processing: %s\n",path.c_str());

	if ( ! boost::filesystem::exists( path ) ){return 0;}

	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	int viewgroup_nrviews = readNumberOfViews(sweep_folder+"ViewGroup.xml");


	printf("sweep_folder: %s\n",sweep_folder.c_str());

	int additional_nrviews = 0;
	QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("*object*.xml"));
	for (auto objectFile : objectFiles){
		printf("objectFile: %s\n",(sweep_folder+objectFile.toStdString()).c_str());
		auto object = loadDynamicObjectFromSingleSweep<PointType>(sweep_folder+objectFile.toStdString(),false);
		additional_nrviews += object.vAdditionalViews.size();
	}

	SimpleXMLParser<pcl::PointXYZRGB> parser;
	SimpleXMLParser<pcl::PointXYZRGB>::RoomData current_roomData  = parser.loadRoomFromXML(path,std::vector<std::string>{"RoomIntermediateCloud","IntermediatePosition"});
	int metaroom_nrviews = current_roomData.vIntermediateRoomClouds.size();


	printf("viewgroup_nrviews: %i\n",viewgroup_nrviews);
	printf("additional_nrviews: %i\n",additional_nrviews);
	printf("metaroom_nrviews: %i\n",metaroom_nrviews);

	reglib::Model * fullmodel;
	if(viewgroup_nrviews == (additional_nrviews+metaroom_nrviews) && !recomputeRelativePoses){
		printf("time to read old\n");
		fullmodel = new reglib::Model();
		fullmodel->savePath = saveVisuals_sp+"/";

		for(unsigned int i = 0; i < viewgroup_nrviews; i++){
			cv::Mat fullmask;
			fullmask.create(480,640,CV_8UC1);
			unsigned char * maskdata = (unsigned char *)fullmask.data;
			for(int j = 0; j < 480*640; j++){maskdata[j] = 255;}
			fullmodel->modelmasks.push_back(new reglib::ModelMask(fullmask));
		}

        readViewXML(sweep_folder+"ViewGroup.xml",fullmodel->frames,fullmodel->relativeposes,compute_edges,saveVisuals_sp);
		fullmodel->recomputeModelPoints();
	}else{
        fullmodel = processAV(path,compute_edges,saveVisuals_sp);
		writeXml(sweep_folder+"ViewGroup.xml",fullmodel->frames,fullmodel->relativeposes);
	}

	return fullmodel;
}

int totalcounter = 0;

int processMetaroom(std::string path, bool store_old_xml = true){

	quasimodo_brain::cleanPath(path);
	int returnval = 0;
	printf("processing: %s\n",path.c_str());

	if ( ! boost::filesystem::exists( path ) ){return 0;}

	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("*object*.xml"));
	store_old_xml = objectFiles.size() == 0;

	printf("current_roomData starting to load\n");

	SimpleXMLParser<pcl::PointXYZRGB> parser;
	SimpleXMLParser<pcl::PointXYZRGB>::RoomData current_roomData  = parser.loadRoomFromXML(path,std::vector<std::string>{"RoomIntermediateCloud","IntermediatePosition"});

	printf("current_roomData loaded\n");

    reglib::Model * fullmodel = getAVMetaroom(path,true,saveVisuals);

	printf("fullmodel->frames.size() = %i\n",fullmodel->frames.size());

//	fullmodel->fullDelete();
//	delete fullmodel;
//	return 0;


	reglib::RegistrationRandom *	reg	= new reglib::RegistrationRandom();
	reglib::ModelUpdaterBasicFuse * mu	= new reglib::ModelUpdaterBasicFuse( fullmodel, reg);
	mu->occlusion_penalty               = 15;
	mu->massreg_timeout                 = 60*4;
	mu->viewer							= viewer;

	std::vector<Eigen::Matrix4d> po;
	std::vector<reglib::RGBDFrame*> fr;
	std::vector<reglib::ModelMask*> mm;
	fullmodel->getData(po, fr, mm);
	//	fullmodel->points = mu->getSuperPoints(po,fr,mm,1,false);
	//	fullmodel->recomputeModelPoints();
	//	SemanticRoom<PointType> observation = SemanticRoomXMLParser<PointType>::loadRoomFromXML(path,false);
	//	Eigen::Matrix4f roomTransform = observation.getRoomTransform();
	//	printf("roomTransform\n");
	//	std::cout << roomTransform << std::endl;

//	return 0;
//exit(0);

	DynamicObjectXMLParser objectparser(sweep_folder, true);

	std::string current_waypointid = current_roomData.roomWaypointId;

	if(overall_folder.back() == '/'){overall_folder.pop_back();}

	int prevind = -1;
	std::vector<std::string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<pcl::PointXYZRGB>(overall_folder);
	for (unsigned int i = 0; i < sweep_xmls.size(); i++){
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData other_roomData  = parser.loadRoomFromXML(sweep_xmls[i],std::vector<std::string>(),false,false);
		std::string other_waypointid = other_roomData.roomWaypointId;

		if(sweep_xmls[i].compare(path) == 0){break;}
		if(other_waypointid.compare(current_waypointid) == 0){prevind = i;}
	}

	int nextind = sweep_xmls.size();
	for (unsigned int i = (sweep_xmls.size()-1); i >= 0 ; i--){
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData other_roomData  = parser.loadRoomFromXML(sweep_xmls[i],std::vector<std::string>(),false,false);
		std::string other_waypointid = other_roomData.roomWaypointId;

		if(sweep_xmls[i].compare(path) == 0){break;}
		if(other_waypointid.compare(current_waypointid) == 0){nextind = i;}
	}

	std::vector< reglib::Model * > models;
	models.push_back(fullmodel);

	std::vector< std::vector< cv::Mat > > internal;
	std::vector< std::vector< cv::Mat > > external;
	std::vector< std::vector< cv::Mat > > dynamic;

	std::vector< reglib::Model * > bgs;
	if(prevind != -1){
		std::string prev = sweep_xmls[prevind];
		printf("prev: %s\n",prev.c_str());
		reglib::Model * bg = getAVMetaroom(prev);
		bgs.push_back(bg);
	}

//	if(nextind < sweep_xmls.size()){
//		std::string next = sweep_xmls[nextind];
//		printf("next: %s\n",next.c_str());
//		reglib::Model * nxt = getAVMetaroom(next);
//		bgs.push_back(nxt);
//	}
printf("bgs.size() = %i\n",bgs.size());
	if(bgs.size() > 0){
		auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(path, std::vector<std::string>{},false);

		printf("models.front()->frames.size() = %i\n",models.front()->frames.size());
		quasimodo_brain::segment(bgs,models,internal,external,dynamic,visualization_lvl > 0,saveVisuals);

		remove_old_seg(sweep_folder);

		if(models.size() == 0){	returnval = 2;}
		else{					returnval = 3;}

		for(unsigned int i = 0; i < models.size(); i++){
			std::vector<cv::Mat> internal_masks = internal[i];
			std::vector<cv::Mat> external_masks = external[i];
			std::vector<cv::Mat> dynamic_masks	= dynamic[i];
			reglib::Model * model = models[i];

			std::vector<Eigen::Matrix4d> mod_po;
			std::vector<reglib::RGBDFrame*> mod_fr;
			std::vector<reglib::ModelMask*> mod_mm;
			model->getData(mod_po, mod_fr, mod_mm);

			int dynamicCounter = 1;
			while(true){
				double sum = 0;
				double sumx = 0;
				double sumy = 0;
				double sumz = 0;
				std::vector<int> imgnr;
				std::vector<cv::Mat> masks;

				CloudPtr cloud_cluster (new Cloud());

				Eigen::Matrix4d first = model->frames.front()->pose;

				for(unsigned int j = 0; j < mod_fr.size(); j++){
					reglib::RGBDFrame * frame = mod_fr[j];
					//std::cout << first*mod_po[j] << std::endl;
					Eigen::Matrix4d p = frame->pose;//first*mod_po[j];//*bg->frames.front()->pose;
					unsigned char  * rgbdata		= (unsigned char	*)(frame->rgb.data);
					unsigned short * depthdata		= (unsigned short	*)(frame->depth.data);
					float		   * normalsdata	= (float			*)(frame->normals.data);

					reglib::Camera * camera = frame->camera;

					unsigned char * internalmaskdata = (unsigned char *)(internal_masks[j].data);
					unsigned char * externalmaskdata = (unsigned char *)(external_masks[j].data);
					unsigned char * dynamicmaskdata = (unsigned char *)(dynamic_masks[j].data);

					float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
					float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
					float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

					const float idepth			= camera->idepth_scale;
					const float cx				= camera->cx;
					const float cy				= camera->cy;
					const float ifx				= 1.0/camera->fx;
					const float ify				= 1.0/camera->fy;
					const unsigned int width	= camera->width;
					const unsigned int height	= camera->height;

					cv::Mat mask;
					mask.create(height,width,CV_8UC1);
					unsigned char * maskdata = (unsigned char *)(mask.data);


					bool containsData = false;
					for(unsigned int w = 0; w < width;w++){
						for(unsigned int h = 0; h < height;h++){
							int ind = h*width+w;

							if(dynamicmaskdata[ind] == dynamicCounter){
								maskdata[ind] = 255;
								containsData = true;
								float z = idepth*float(depthdata[ind]);
								if(z > 0){
									float x = (float(w) - cx) * z * ifx;
									float y = (float(h) - cy) * z * ify;

									PointType p;
									p.x += m00*x + m01*y + m02*z + m03;
									p.y += m10*x + m11*y + m12*z + m13;
									p.z += m20*x + m21*y + m22*z + m23;
									p.r  = rgbdata[3*ind+2];
									p.g  = rgbdata[3*ind+1];
									p.b  = rgbdata[3*ind+0];
									cloud_cluster->points.push_back (p);

									sumx += p.x;
									sumy += p.y;
									sumz += p.z;
									sum ++;
								}
							}else{
								maskdata[ind] = 0;
							}
						}
					}

					if(containsData){
						masks.push_back(mask);
						imgnr.push_back(j);
					}
				}

				cloud_cluster->width = cloud_cluster->points.size ();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;

				if(masks.size() > 0){

					if(store_old_xml){
						std::stringstream ss;
						ss << "object_";
						ss << (dynamicCounter-1);

						// create and save dynamic object
						DynamicObject::Ptr roomObject(new DynamicObject());
						roomObject->setCloud(cloud_cluster);
						roomObject->setTime(sweep.roomLogStartTime);
						roomObject->m_roomLogString = sweep.roomLogName;
						roomObject->m_roomStringId = sweep.roomWaypointId;
						roomObject->m_roomRunNumber = sweep.roomRunNumber;
						//				        // create label from room log time; could be useful later on, and would resolve ambiguities
						std::stringstream ss_obj;
						ss_obj<<boost::posix_time::to_simple_string(sweep.roomLogStartTime);
						ss_obj<<"_object_";ss_obj<<(dynamicCounter-1);
						roomObject->m_label = ss_obj.str();
						std::string xml_file = objectparser.saveAsXML(roomObject);
						printf("xml_file: %s\n",xml_file.c_str());
					}

					char buf [1024];
					sprintf(buf,"%s/dynamic_obj%10.10i.pcd",sweep_folder.c_str(),dynamicCounter-1);
					pcl::io::savePCDFileBinaryCompressed(std::string(buf),*cloud_cluster);


					//					std::string objectpcd = std::string(buf);//object.substr(0,object.size()-4);
					//					std::cout << objectpcd.substr(0,objectpcd.size()-4) << std::endl;

					sprintf(buf,"%s/dynamic_obj%10.10i.xml",sweep_folder.c_str(),dynamicCounter-1);
					printf("saving dynamic objec: %s\n",buf);
					QFile file(buf);
					if (file.exists()){file.remove();}
					if (!file.open(QIODevice::ReadWrite | QIODevice::Text)){std::cerr<<"Could not open file "<< buf <<" to save dynamic object as XML"<<std::endl;}
					QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
					xmlWriter->setDevice(&file);

					xmlWriter->writeStartDocument();
					xmlWriter->writeStartElement("Object");
					xmlWriter->writeAttribute("object_number", QString::number(dynamicCounter-1));
					xmlWriter->writeAttribute("classname", QString(""));
					xmlWriter->writeAttribute("instancename", QString(""));
					xmlWriter->writeAttribute("tags", QString(""));
					xmlWriter->writeStartElement("Mean");
					xmlWriter->writeAttribute("x", QString::number(sumx/sum));
					xmlWriter->writeAttribute("y", QString::number(sumy/sum));
					xmlWriter->writeAttribute("z", QString::number(sumz/sum));
					xmlWriter->writeEndElement();


					for(unsigned int j = 0; j < masks.size(); j++){
						char buf [1024];
						sprintf(buf,"%s/dynamicmask_%i_%i.png",sweep_folder.c_str(),dynamicCounter-1,imgnr[j]);
						cv::imwrite(buf, masks[j] );

						int width = masks[j].cols;
						int height = masks[j].rows;

						int maxw = 0;
						int minw = width;
						int maxh = 0;
						int minh = height;
						for(int w = 0; w < width; w++){
							for(int h = 0; h < height; h++){
								if(masks[j].data[h*width+w] != 0){
									maxw = std::max(maxw,w);
									minw = std::min(minw,w);
									maxh = std::max(maxh,h);
									minh = std::min(minh,h);
								}
							}
						}

						double ratio = 0.15;
						int diffw = maxw-minw;
						int diffh = maxh-minh;

						maxw = std::min(width-1	,int(maxw+ratio*diffw));
						minw = std::max(0		,int(minw-ratio*diffw));
						maxh = std::min(height-1,int(maxh+ratio*diffh));
						minh = std::max(0		,int(minh-ratio*diffh));

						diffw = maxw-minw;
						diffh = maxh-minh;

						cv::Mat image = mod_fr[imgnr[j]]->rgb.clone();
						cv::Rect myROI(minw, minh, diffw, diffh);
						cv::Mat localimg = image(myROI);


						char buf2 [1024];
						sprintf(buf2,"/home/johane/imgregion/region%10.10i.png",totalcounter++);
						cv::imwrite(buf2, localimg );

						printf("saving dynamic mask: dynamicmask_%i_%i.png\n",dynamicCounter-1,imgnr[j]);

						sprintf(buf,"dynamicmask_%i_%i.png",dynamicCounter-1,imgnr[j]);
						xmlWriter->writeStartElement("Mask");
						xmlWriter->writeAttribute("filename", QString(buf));
						xmlWriter->writeAttribute("image_number", QString::number(imgnr[j]));
						xmlWriter->writeEndElement();
					}

					xmlWriter->writeEndElement();
					xmlWriter->writeEndDocument();
					delete xmlWriter;
					dynamicCounter++;
				}else{break;}
			}

			int movingCounter = 1;
			while(true){
				double sum = 0;
				double sumx = 0;
				double sumy = 0;
				double sumz = 0;
				std::vector<int> imgnr;
				std::vector<cv::Mat> masks;

				for(unsigned int j = 0; j < mod_fr.size(); j++){
					reglib::RGBDFrame * frame = mod_fr[j];
					//std::cout << mod_po[j] << std::endl;
					Eigen::Matrix4d p = mod_po[j]*bgs.front()->frames.front()->pose;
					unsigned char  * rgbdata		= (unsigned char	*)(frame->rgb.data);
					unsigned short * depthdata		= (unsigned short	*)(frame->depth.data);
					float		   * normalsdata	= (float			*)(frame->normals.data);

					reglib::Camera * camera = frame->camera;

					unsigned char * internalmaskdata = (unsigned char *)(internal_masks[j].data);
					unsigned char * externalmaskdata = (unsigned char *)(external_masks[j].data);
					unsigned char * dynamicmaskdata = (unsigned char *)(dynamic_masks[j].data);

					float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
					float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
					float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

					const float idepth			= camera->idepth_scale;
					const float cx				= camera->cx;
					const float cy				= camera->cy;
					const float ifx				= 1.0/camera->fx;
					const float ify				= 1.0/camera->fy;
					const unsigned int width	= camera->width;
					const unsigned int height	= camera->height;

					cv::Mat mask;
					mask.create(height,width,CV_8UC1);
					unsigned char * maskdata = (unsigned char *)(mask.data);

					bool containsData = false;
					for(unsigned int w = 0; w < width;w++){
						for(unsigned int h = 0; h < height;h++){
							int ind = h*width+w;

							if(internalmaskdata[ind] == movingCounter){
								maskdata[ind] = 255;
								containsData = true;
								float z = idepth*float(depthdata[ind]);
								if(z > 0){
									float x = (float(w) - cx) * z * ifx;
									float y = (float(h) - cy) * z * ify;

									sumx += m00*x + m01*y + m02*z + m03;
									sumy += m10*x + m11*y + m12*z + m13;
									sumz += m20*x + m21*y + m22*z + m23;
									sum ++;
								}
							}else{
								maskdata[ind] = 0;
							}
						}
					}

					if(containsData){
						masks.push_back(mask);
						imgnr.push_back(j);
					}
				}
				if(masks.size() > 0){
					char buf [1024];
					sprintf(buf,"%s/moving_obj%10.10i.xml",sweep_folder.c_str(),movingCounter-1);
					QFile file(buf);
					if (file.exists()){file.remove();}
					if (!file.open(QIODevice::ReadWrite | QIODevice::Text)){std::cerr<<"Could not open file "<< buf <<" to save dynamic object as XML"<<std::endl;}
					QXmlStreamWriter* xmlWriter = new QXmlStreamWriter();
					xmlWriter->setDevice(&file);

					xmlWriter->writeStartDocument();
					xmlWriter->writeStartElement("Object");
					xmlWriter->writeAttribute("object_number", QString::number(movingCounter-1));
					xmlWriter->writeAttribute("label", QString(""));
					xmlWriter->writeStartElement("Mean");
					xmlWriter->writeAttribute("x", QString::number(sumx/sum));
					xmlWriter->writeAttribute("y", QString::number(sumy/sum));
					xmlWriter->writeAttribute("z", QString::number(sumz/sum));
					xmlWriter->writeEndElement();

					for(unsigned int j = 0; j < masks.size(); j++){
						char buf [1024];
						sprintf(buf,"%s/movingmask_%i_%i.png",sweep_folder.c_str(),movingCounter-1,imgnr[j]);
						cv::imwrite(buf, masks[j] );
						sprintf(buf,"movingmask_%i_%i.png",movingCounter-1,imgnr[j]);
						xmlWriter->writeStartElement("Mask");
						xmlWriter->writeAttribute("filename", QString(buf));
						xmlWriter->writeAttribute("image_number", QString::number(imgnr[j]));
						xmlWriter->writeEndElement();
					}

					xmlWriter->writeEndElement();
					xmlWriter->writeEndDocument();
					delete xmlWriter;
					movingCounter++;
				}else{break;}
			}
		}
	}else{
		returnval = 1;
	}

	for(unsigned int i = 0; i < bgs.size(); i++){
		bgs[i]->fullDelete();
		delete bgs[i];
	}

	fullmodel->fullDelete();
	delete fullmodel;
	//	delete bgmassreg;
	delete reg;
	delete mu;
	printf("publishing file %s to %s\n",path.c_str(),outtopic.c_str());

	std_msgs::String msg;
	msg.data = path;
	for(unsigned int i = 0; i < out_pubs.size(); i++){out_pubs[i].publish(msg);}
	ros::spinOnce();

	return returnval;
}

void chatterCallback(const std_msgs::String::ConstPtr& msg){processMetaroom(msg->data);}

void trainMetaroom(std::string path){
	printf("processing: %s\n",path.c_str());
	if(posepath.compare("")==0){
		printf("posepath not set, set before training\n");
		return ;
	}

	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	reglib::Model * fullmodel = processAV(path);

	//Not needed if metaroom well calibrated
	reglib::MassRegistrationPPR2 * bgmassreg = new reglib::MassRegistrationPPR2(0.01);
	bgmassreg->timeout = 600;
	bgmassreg->viewer = viewer;
	bgmassreg->use_surface = true;
	bgmassreg->use_depthedge = true;
	bgmassreg->visualizationLvl = visualization_lvl;//0;
	bgmassreg->maskstep = 5;
	bgmassreg->nomaskstep = 5;
	bgmassreg->nomask = true;
	bgmassreg->stopval = 0.0005;
	bgmassreg->setData(fullmodel->frames,fullmodel->modelmasks);
	reglib::MassFusionResults bgmfr = bgmassreg->getTransforms(fullmodel->relativeposes);

	savePoses(overall_folder+"/"+posepath,bgmfr.poses,17);
	fullmodel->fullDelete();
	delete fullmodel;
	delete bgmassreg;
}

std::vector<reglib::Model *> loadModels(std::string path){
	printf("loadModels: %s\n",path.c_str());
	std::vector<reglib::Model *> models;
	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	std::vector<reglib::RGBDFrame *> frames;
	std::vector<Eigen::Matrix4d> poses;
	readViewXML(sweep_folder+"ViewGroup.xml",frames,poses);

	QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("dynamic_obj*.xml"));
	for (auto objectFile : objectFiles){
		std::string object = sweep_folder+objectFile.toStdString();
		printf("object: %s\n",object.c_str());

		QFile file(object.c_str());

		if (!file.exists()){
			ROS_ERROR("Could not open file %s to masks.",object.c_str());
			continue;
		}

		file.open(QIODevice::ReadOnly);
		ROS_INFO_STREAM("Parsing xml file: "<<object.c_str());

		reglib::Model * mod = new reglib::Model();
		QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);

		while (!xmlReader->atEnd() && !xmlReader->hasError()){
			QXmlStreamReader::TokenType token = xmlReader->readNext();
			if (token == QXmlStreamReader::StartDocument)
				continue;

			if (xmlReader->hasError()){
				ROS_ERROR("XML error: %s",xmlReader->errorString().toStdString().c_str());
				break;
			}

			QString elementName = xmlReader->name().toString();

			if (token == QXmlStreamReader::StartElement){
				if (xmlReader->name() == "Mask"){
					int number = 0;
					cv::Mat mask;
					QXmlStreamAttributes attributes = xmlReader->attributes();
					if (attributes.hasAttribute("filename")){
						QString maskpath = attributes.value("filename").toString();
						//printf("mask filename: %s\n",(sweep_folder+maskpath.toStdString()).c_str());
						mask = cv::imread(sweep_folder+"/"+(maskpath.toStdString().c_str()), CV_LOAD_IMAGE_UNCHANGED);
						//mask = cv::imread((maskpath.toStdString()).c_str(), CV_LOAD_IMAGE_UNCHANGED);
					}else{break;}


					if (attributes.hasAttribute("image_number")){
						QString depthpath = attributes.value("image_number").toString();
						number = atoi(depthpath.toStdString().c_str());
						printf("number: %i\n",number);
					}else{break;}

					mod->frames.push_back(frames[number]->clone());
					mod->relativeposes.push_back(poses[number]);
					mod->modelmasks.push_back(new reglib::ModelMask(mask));
				}
			}
		}

		delete xmlReader;
		models.push_back(mod);
	}

	for(unsigned int i = 0; i < frames.size(); i++){delete frames[i];}

	return models;
}

void addModelToModelServer(reglib::Model * model){
	printf("addModelToModelServer\n");
	for(unsigned int i = 0; i < model_pubs.size(); i++){model_pubs[i].publish(quasimodo_brain::getModelMSG(model));}
	ros::spinOnce();
}

void sendMetaroomToServer(std::string path){
	std::vector<reglib::Model *> mods = loadModels(path);
	for(unsigned int i = 0; i < mods.size(); i++){
		addModelToModelServer(mods[i]);
		mods[i]->fullDelete();
		delete mods[i];
	}
}


void sendCallback(const std_msgs::String::ConstPtr& msg){sendMetaroomToServer(msg->data);}

bool dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res){
	printf("bool dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res)\n");
	std::string current_waypointid = req.waypoint_id;

	printf("current_waypointid: %i\n",current_waypointid.c_str());


	if(overall_folder.back() == '/'){overall_folder.pop_back();}

	SimpleXMLParser<pcl::PointXYZRGB> parser;
	int prevind = -1;
	std::vector<std::string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<pcl::PointXYZRGB>(overall_folder);
	for (unsigned int i = 0; i < sweep_xmls.size(); i++){
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData other_roomData  = parser.loadRoomFromXML(sweep_xmls[i],std::vector<std::string>(),false,false);
		std::string other_waypointid = other_roomData.roomWaypointId;
		if(other_waypointid.compare(current_waypointid) == 0){prevind = i;}
	}
	if(prevind == -1){return false;}
	std::string path = sweep_xmls[prevind];
	//processMetaroom(path);

	printf("path: %s\n",path.c_str());
	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	printf("sweep_folder: %s\n",sweep_folder.c_str());

	QStringList objectFiles = QDir(sweep_folder.c_str()).entryList(QStringList("dynamic_object*.xml"));
	for (auto objectFile : objectFiles){
		std::string object = sweep_folder+objectFile.toStdString();
		printf("object: %s\n",object.c_str());

		QFile file(object.c_str());

		if (!file.exists()){
			ROS_ERROR("Could not open file %s to masks.",object.c_str());
			continue;
		}

		file.open(QIODevice::ReadOnly);
		ROS_INFO_STREAM("Parsing xml file: "<<object.c_str());

		QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);

		while (!xmlReader->atEnd() && !xmlReader->hasError()){
			QXmlStreamReader::TokenType token = xmlReader->readNext();
			if (token == QXmlStreamReader::StartDocument)
				continue;

			if (xmlReader->hasError()){
				ROS_ERROR("XML error: %s",xmlReader->errorString().toStdString().c_str());
				break;
			}

			QString elementName = xmlReader->name().toString();
			if (token == QXmlStreamReader::StartElement){
				if (xmlReader->name() == "Mean"){
					QXmlStreamAttributes attributes = xmlReader->attributes();
					double x = atof(attributes.value("x").toString().toStdString().c_str());
					double y = atof(attributes.value("y").toString().toStdString().c_str());
					double z = atof(attributes.value("z").toString().toStdString().c_str());
					printf("mean: %f %f %f\n",x,y,z);

					std::string objectpcd = object.substr(0,object.size()-4);
					std::cout << objectpcd << std::endl;

					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::io::loadPCDFile<pcl::PointXYZRGB> (objectpcd+".pcd", *cloud);

					printf("cloud->points.size() = %i\n",cloud->points.size());

					sensor_msgs::PointCloud2 msg_objects;
					pcl::toROSMsg(*cloud, msg_objects);
					msg_objects.header.frame_id="/map";

					geometry_msgs::Point p;
					p.x = x;
					p.y = y;
					p.z = z;

					res.object_id.push_back(object);
					res.objects.push_back(msg_objects);
					res.centroids.push_back(p);
				}
			}
		}
		delete xmlReader;
	}

	return true;
}

bool getDynamicObjectServiceCallback(GetDynamicObjectServiceRequest &req, GetDynamicObjectServiceResponse &res){
	std::string current_waypointid = req.waypoint_id;

	if(overall_folder.back() == '/'){overall_folder.pop_back();}

	SimpleXMLParser<pcl::PointXYZRGB> parser;
	int prevind = -1;
	std::vector<std::string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<pcl::PointXYZRGB>(overall_folder);
	for (unsigned int i = 0; i < sweep_xmls.size(); i++){
		SimpleXMLParser<pcl::PointXYZRGB>::RoomData other_roomData  = parser.loadRoomFromXML(sweep_xmls[i],std::vector<std::string>(),false,false);
		std::string other_waypointid = other_roomData.roomWaypointId;
		if(other_waypointid.compare(current_waypointid) == 0){prevind = i;}
	}
	if(prevind == -1){return false;}
	std::string path = sweep_xmls[prevind];

	printf("path: %s\n",path.c_str());
	int slash_pos = path.find_last_of("/");
	std::string sweep_folder = path.substr(0, slash_pos) + "/";

	printf("sweep_folder: %s\n",sweep_folder.c_str());
	printf("req.object_id: %s\n",req.object_id.c_str());
	char buf [1024];
	std::string fullpath = std::string(buf);

	return true;
}

bool segmentRaresFiles(std::string path, bool resegment){
	printf("bool segmentRaresFiles(%s)\n",path.c_str());

	vector<string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(path);
	for (auto sweep_xml : sweep_xmls) {
		printf("sweep_xml: %s\n",sweep_xml.c_str());

		quasimodo_brain::cleanPath(sweep_xml);
		int slash_pos = sweep_xml.find_last_of("/");
		std::string sweep_folder = sweep_xml.substr(0, slash_pos) + "/";
		QStringList segoutput = QDir(sweep_folder.c_str()).entryList(QStringList("segoutput.txt"));
		//QStringList movFiles = QDir(sweep_folder.c_str()).entryList(QStringList("*moving_obj*.xml"));

		//printf("sweep_folder: %s\n",sweep_folder.c_str());
		printf("segoutput %i\n",segoutput.size());
		if(resegment || segoutput.size() == 0){

			std::ofstream myfile;
			myfile.open (sweep_folder+"segoutput.txt");
			myfile << "dummy";
			myfile.close();


			processMetaroom(sweep_xml);
		}

	}
	return false;
}

bool testDynamicObjectServiceCallback(std::string path){
	printf("bool getDynamicObjectServiceCallback(GetDynamicObjectServiceRequest &req, GetDynamicObjectServiceResponse &res)\n");
	DynamicObjectsServiceRequest req;
	req.waypoint_id = path;
	DynamicObjectsServiceResponse res;
	return dynamicObjectsServiceCallback(req,res);
}

void roomObservationCallback(const semantic_map_msgs::RoomObservationConstPtr& obs_msg) {
	std::cout<<"Room obs message received"<<std::endl;
	int ret = processMetaroom(obs_msg->xml_file_name);

	std_msgs::String msg;
	if(ret == 0){
		ROS_ERROR_STREAM("Xml file does not exist. Aborting.");
		msg.data	= "error_processing_observation";
	}else{msg.data	= "finished_processing_observation";}

	if(ret == 1){ROS_ERROR_STREAM("First metaroom.");}
	if(ret == 2){ROS_ERROR_STREAM("No moving objects found.");}
	if(ret == 3){ROS_ERROR_STREAM("Moving objects found.");}

	for(unsigned int i = 0; i < m_PublisherStatuss.size(); i++){m_PublisherStatuss[i].publish(msg);}
}


void setLargeStack(){
	const rlim_t kStackSize = 256 * 1024 * 1024;   // min stack size = 256 MB
	struct rlimit rl;
	unsigned long result;

	result = getrlimit(RLIMIT_STACK, &rl);
	if (result == 0){
		if (rl.rlim_cur < kStackSize){
			rl.rlim_cur = kStackSize;
			result = setrlimit(RLIMIT_STACK, &rl);
			if (result != 0){fprintf(stderr, "setrlimit returned result = %d\n", int(result));}
		}
	}
}

int main(int argc, char** argv){
	bool baseSetting = true;
	bool once = false;

	overall_folder	= std::string(getenv ("HOME"))+"/.semanticMap/";
	outtopic		= "/some/topic";
	modelouttopic	= "/model/out/topic";
	posepath		= "testposes.xml";

	ros::init(argc, argv, "metaroom_additional_view_processing");
	ros::NodeHandle n;

	std::vector<ros::Subscriber> segsubs;
	std::vector<ros::Subscriber> sendsubs;
	std::vector<ros::Subscriber> roomObservationSubs;

	std::vector<std::string> trainMetarooms;
	std::vector<std::string> sendMetaroomToServers;
	std::vector<std::string> processMetarooms;

	std::vector<bool>		 raresfiles_resegment;
	std::vector<std::string> raresfiles;

	bool resegment					= false;

	int inputstate = 0;
	for(int i = 1; i < argc;i++){
		printf("input: %s\n",argv[i]);
		if(		std::string(argv[i]).compare("-intopic") == 0){		inputstate = 0;}
		else if(std::string(argv[i]).compare("-outtopic") == 0){	inputstate = 1;}
		else if(std::string(argv[i]).compare("-file") == 0){		inputstate = 2;}
		else if(std::string(argv[i]).compare("-v") == 0){	visualization_lvl = 1;inputstate = 3;}
		else if(std::string(argv[i]).compare("-folder") == 0){					inputstate	= 4;}
		else if(std::string(argv[i]).compare("-train") == 0){					inputstate	= 5;}
		else if(std::string(argv[i]).compare("-posepath") == 0){				inputstate	= 6;}
		else if(std::string(argv[i]).compare("-loadposes") == 0){				inputstate	= 7;}
		else if(std::string(argv[i]).compare("-sendModel") == 0){				inputstate	= 8;}
		else if(std::string(argv[i]).compare("-sendSub") == 0)	{				inputstate	= 9;}
		else if(std::string(argv[i]).compare("-sendTopic") == 0){				inputstate	= 10;}
		else if(std::string(argv[i]).compare("-roomObservationTopic") == 0){	inputstate	= 11;}
		else if(std::string(argv[i]).compare("-DynamicObjectsService") == 0){	inputstate	= 12;}
		else if(std::string(argv[i]).compare("-GetDynamicObjectService") == 0){	inputstate	= 13;}
		else if(std::string(argv[i]).compare("-statusmsg") == 0){				inputstate	= 14;}
		else if(std::string(argv[i]).compare("-files") == 0){					inputstate	= 15;}
		else if(std::string(argv[i]).compare("-baseSweep") == 0){				inputstate	= 16;}
		else if(std::string(argv[i]).compare("-resegment") == 0){				resegment	= true;}
		else if(std::string(argv[i]).compare("-once") == 0){					once		= true;}
		else if(std::string(argv[i]).compare("-nobase") == 0){					baseSetting = false;}
		else if(std::string(argv[i]).compare("-recomputeRelativePoses") == 0){	recomputeRelativePoses = true;}
        else if(std::string(argv[i]).compare("-v_init") == 0){	visualization_lvl_regini = 1;inputstate = 17;}
        else if(std::string(argv[i]).compare("-v_reg") == 0){	visualization_lvl_regref = 1;inputstate = 18;}
        else if(std::string(argv[i]).compare("-saveVisuals") == 0){inputstate = 19;}
		else if(inputstate == 0){
			segsubs.push_back(n.subscribe(std::string(argv[i]), 1000, chatterCallback));
		}else if(inputstate == 1){
			out_pubs.push_back(n.advertise<std_msgs::String>(std::string(argv[i]), 1000));
		}else if(inputstate == 2){
			processMetarooms.push_back(std::string(argv[i]));
		}else if(inputstate == 3){
			visualization_lvl = atoi(argv[i]);
		}else if(inputstate == 4){
			overall_folder = std::string(argv[i]);
		}else if(inputstate == 5){
			trainMetarooms.push_back(std::string(argv[i]));
		}else if(inputstate == 6){
			posepath = std::string(argv[i]);
		}else if(inputstate == 7){
			sweepPoses = readPoseXML(std::string(argv[i]));
		}else if(inputstate == 8){
			sendMetaroomToServers.push_back(std::string(argv[i]));
		}else if(inputstate == 9){
			sendsubs.push_back(n.subscribe(std::string(argv[i]), 1000, sendCallback));
		}else if(inputstate == 10){
			model_pubs.push_back(n.advertise<quasimodo_msgs::model>(std::string(argv[i]), 1000));
		}else if(inputstate == 11){
			roomObservationSubs.push_back(n.subscribe(std::string(argv[i]), 1000, roomObservationCallback));
		}else if(inputstate == 12){
			m_DynamicObjectsServiceServers.push_back(n.advertiseService(std::string(argv[i]), dynamicObjectsServiceCallback));
		}else if(inputstate == 13){
			m_GetDynamicObjectServiceServers.push_back(n.advertiseService(std::string(argv[i]), getDynamicObjectServiceCallback));
		}else if(inputstate == 14){
			m_PublisherStatuss.push_back(n.advertise<std_msgs::String>(std::string(argv[i]), 1000));
		}else if(inputstate == 15){
			raresfiles.push_back(std::string(argv[i]));
			raresfiles_resegment.push_back(resegment);
		}else if(inputstate == 16){
			setBaseSweep(std::string(argv[i]));
		}else if(inputstate == 17){
			visualization_lvl_regini = atoi(argv[i]);
        }else if(inputstate == 18){
            visualization_lvl_regref = atoi(argv[i]);
        }else if(inputstate == 19){
            saveVisuals = std::string(argv[i]);
        }
	}

	if(visualization_lvl != 0 || visualization_lvl_regref != 0 || visualization_lvl_regini != 0){
		viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0.5, 0, 0.5);
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
	}

	if(baseSetting){
		if(m_PublisherStatuss.size() == 0){
			m_PublisherStatuss.push_back(n.advertise<std_msgs::String>("/local_metric_map/status", 1000));
		}

		if(segsubs.size() == 0){
			segsubs.push_back(n.subscribe("/some/inputtopic", 1000, chatterCallback));
		}

		if(out_pubs.size() == 0){
			out_pubs.push_back(n.advertise<std_msgs::String>("/some/topic", 1000));
		}

		if(model_pubs.size() == 0){
			model_pubs.push_back(n.advertise<quasimodo_msgs::model>("/model/out/topic", 1000));
		}

		if(sendsubs.size() == 0){
			sendsubs.push_back(n.subscribe("/model/out/topic/path", 1000, sendCallback));
		}

		if(roomObservationSubs.size() == 0){
			roomObservationSubs.push_back(n.subscribe("/local_metric_map/room_observations", 1000, roomObservationCallback));
		}
	}

	printf("overall_folder: %s\n",overall_folder.c_str());

	for(unsigned int i = 0; i < raresfiles.size(); i++){			segmentRaresFiles(		raresfiles[i], raresfiles_resegment[i]);}
	for(unsigned int i = 0; i < trainMetarooms.size(); i++){		trainMetaroom(			trainMetarooms[i]);}
	for(unsigned int i = 0; i < processMetarooms.size(); i++){		processMetaroom(		processMetarooms[i]);}
	for(unsigned int i = 0; i < sendMetaroomToServers.size(); i++){	sendMetaroomToServer(	sendMetaroomToServers[i]);}

	if(!once){ros::spin();}
	printf("done...\n");
	return 0;
}
