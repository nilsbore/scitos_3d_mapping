#include "Util/Util.h"
#include <semantic_map/room_xml_parser.h>

#include <calibrate_sweeps/CalibrateSweepsAction.h>
#include <actionlib/server/simple_action_server.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <boost/filesystem.hpp>
#include <tf_conversions/tf_eigen.h>

#include <semantic_map/room_xml_parser.h>
#include <semantic_map/reg_features.h>
#include <semantic_map/room_utilities.h>
#include <semantic_map/reg_transforms.h>
#include <semantic_map/sweep_parameters.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <strands_sweep_registration/RobotContainer.h>

#include "ros/ros.h"
#include "std_msgs/String.h"


using namespace std;
using namespace semantic_map_load_utilties;
typedef pcl::PointXYZRGB PointType;

std::string getRoomFolder(std::string xmlFile){
    QString xmlFileQS(xmlFile.c_str());
    int index = xmlFileQS.lastIndexOf('/');
    return xmlFileQS.left(index).toStdString();
}

void do_lower(std::string xmlFile, std::string saveLocation){
	std::string room = getRoomFolder(xmlFile);
	printf("room: %s\n",room.c_str());
//exit(0);
//QFile file(xmlFile.c_str());
//if (file.exists()){file.remove();}
	SemanticRoomXMLParser<PointType> reg_parser (saveLocation);
	SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(xmlFile,true);

	int counter = 0;
//	while(true){
//		char buf [1024];
//		sprintf(buf,"%s/intermediate_cloud%4.4i.pcd",room.c_str(),counter);

//		printf("path: %s\n",buf);
//		QFile file(buf);

//		if (file.exists()){
//			if(counter > 16){
//				printf("removing\n");
//				file.remove();
//			}
//		}else{
//			break;
//		}
//		counter++;
//	}

	aRoom.getIntermediateCloudTransforms().resize(17);
	aRoom.getIntermdiatePositionImages().resize(17);

	auto a0 = aRoom.getIntermediateCloudTransforms();
	auto a1 = aRoom.getIntermediateCloudTransformsRegistered();
//	auto a2 = aRoom.getIntermediateCloudCameraParameters();
	printf("aRoom.m_SweepParameters.getNumberOfIntermediatePositions() %i\n",aRoom.m_SweepParameters.getNumberOfIntermediatePositions());
	auto a3 = aRoom.getIntermediateCloudCameraParametersCorrected();
	auto a2 = aRoom.getIntermediateCloudCameraParameters();
	aRoom.m_SweepParameters = SweepParameters (-160, 20, 160, -30, 30, -30);
	printf("aRoom.m_SweepParameters.getNumberOfIntermediatePositions() %i\n",aRoom.m_SweepParameters.getNumberOfIntermediatePositions());

//	aRoom.clearIntermediateCloudTransforms();
	aRoom.clearIntermediateCloudRegisteredTransforms();
//	aRoom.clearIntermediateCloudCameraParameters();
//	aRoom.clearIntermediateCloudCameraParametersCorrecteds();

	//std::vector<std::string> paths = aRoom.getIntermediateCloudsFilenames();
	//SemanticRoom<PointType> aRoom2;
	for (size_t i=0; i < 17; i++){
		char buf [1024];
//		aRoom.addIntermediateCloudTransforms(a0[i]);
		aRoom.addIntermediateRoomCloudRegisteredTransform(a1[i]);
//		aRoom.addIntermediateCloudCameraParameters(a2[i]);
		aRoom.addIntermediateCloudCameraParametersCorrected(a3[i]);
		//aRoom2.addIntermediateRoomCloud(paths[i],a0[i],a2[i]);
	}
	aRoom.getIntermediateClouds().resize(17);
	semantic_map_room_utilities::rebuildRegisteredCloud<PointType>(aRoom);
	string room_path = reg_parser.saveRoomAsXML(aRoom,"room.xml",true);

	counter = 0;
	while(true){
		char buf [1024];
		sprintf(buf,"%s/intermediate_cloud%4.4i.pcd",room.c_str(),counter);

		QFile file(buf);

		if (file.exists()){
			if(counter > 16){
				printf("removing %s\n",buf);
				file.remove();
			}
		}else{
			break;
		}
		counter++;
	}

	ROS_INFO_STREAM("..done");
}



int main(int argc, char** argv){
	for(int i = 1; i < argc;i++){
		printf("input: %s\n",argv[i]);
		vector<string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(std::string(argv[i]));
		for(int j = 0; j < sweep_xmls.size(); j++){
			do_lower(sweep_xmls[j],std::string(argv[i]));
		}
	}
	return 0;
}
