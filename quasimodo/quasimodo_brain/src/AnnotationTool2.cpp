#include "Util/Util.h"

#include <metaroom_detections/metaroom_detections.h>

using namespace std;
using namespace semantic_map_load_utilties;
typedef pcl::PointXYZRGB PointType;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
int visualization_lvl		= 0;

std::string getRoomFolder(std::string xmlFile){
	QString xmlFileQS(xmlFile.c_str());
	int index = xmlFileQS.lastIndexOf('/');
	return xmlFileQS.left(index).toStdString();
}

std::vector<cv::Mat> getRGBvec(std::string xmlFile){
	std::vector<cv::Mat> rgbs;
	std::string roomFolder = getRoomFolder(xmlFile);

	QFile file((roomFolder+"/ViewGroup.xml").c_str());
	if (!file.exists()){
		ROS_ERROR("Could not open file %s to load room.",(roomFolder+"/ViewGroup.xml").c_str());
		return rgbs;
	}

	file.open(QIODevice::ReadOnly);

	//ROS_INFO_STREAM("Parsing xml file: "<<(roomFolder+"/ViewGroup.xml").c_str());

	QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);

	while (!xmlReader->atEnd() && !xmlReader->hasError()){
		QXmlStreamReader::TokenType token = xmlReader->readNext();
		if (token == QXmlStreamReader::StartDocument)
			continue;

		if (xmlReader->hasError()){
			ROS_ERROR("XML error: %s in %s",xmlReader->errorString().toStdString().c_str(), (roomFolder+"/ViewGroup.xml").c_str());
			return rgbs;
		}

		QString elementName = xmlReader->name().toString();

		if (token == QXmlStreamReader::StartElement){
			if (xmlReader->name() == "View"){
				QXmlStreamAttributes attributes = xmlReader->attributes();
				if (attributes.hasAttribute("RGB")){
					 rgbs.push_back(cv::imread(roomFolder+"/"+attributes.value("RGB").toString().toStdString(), CV_LOAD_IMAGE_UNCHANGED));
				}else{break;}
			}
		}
	}
	delete xmlReader;
	return rgbs;
}

std::vector<cv::Mat> getDvec(std::string xmlFile){
	std::vector<cv::Mat> rgbs;
	std::string roomFolder = getRoomFolder(xmlFile);

	QFile file((roomFolder+"/ViewGroup.xml").c_str());
	if (!file.exists()){
		ROS_ERROR("Could not open file %s to load room.",(roomFolder+"/ViewGroup.xml").c_str());
		return rgbs;
	}

	file.open(QIODevice::ReadOnly);

	//ROS_INFO_STREAM("Parsing xml file: "<<(roomFolder+"/ViewGroup.xml").c_str());

	QXmlStreamReader* xmlReader = new QXmlStreamReader(&file);

	while (!xmlReader->atEnd() && !xmlReader->hasError()){
		QXmlStreamReader::TokenType token = xmlReader->readNext();
		if (token == QXmlStreamReader::StartDocument)
			continue;

		if (xmlReader->hasError()){
			ROS_ERROR("XML error: %s in %s",xmlReader->errorString().toStdString().c_str(), (roomFolder+"/ViewGroup.xml").c_str());
			return rgbs;
		}

		QString elementName = xmlReader->name().toString();

		if (token == QXmlStreamReader::StartElement){
			if (xmlReader->name() == "View"){
				QXmlStreamAttributes attributes = xmlReader->attributes();
				if (attributes.hasAttribute("DEPTH")){
					 rgbs.push_back(cv::imread(roomFolder+"/"+attributes.value("DEPTH").toString().toStdString(), CV_LOAD_IMAGE_UNCHANGED));
				}else{break;}
			}
		}
	}
	delete xmlReader;
	return rgbs;
}

void getQuasimodoObject(std::string object, std::vector<cv::Mat > & objectMasks, std::vector<unsigned int > & imgNumber){

	std::string roomFolder = getRoomFolder(object);
	QFile objfile(object.c_str());

	if (!objfile.exists()){
		ROS_ERROR("Could not open file %s to masks.",object.c_str());
		return;
	}

	objfile.open(QIODevice::ReadOnly);
	//ROS_INFO_STREAM("Parsing xml file: "<<object.c_str());

	QXmlStreamReader* objxmlReader = new QXmlStreamReader(&objfile);

	while (!objxmlReader->atEnd() && !objxmlReader->hasError()){
		QXmlStreamReader::TokenType token = objxmlReader->readNext();
		if (token == QXmlStreamReader::StartDocument)
			continue;

		if (objxmlReader->hasError()){
			ROS_ERROR("XML error: %s in %s",objxmlReader->errorString().toStdString().c_str(),object.c_str());
			break;
		}

		QString elementName = objxmlReader->name().toString();

		if (token == QXmlStreamReader::StartElement){
			if (objxmlReader->name() == "Mask"){
				int number = 0;
				QXmlStreamAttributes attributes = objxmlReader->attributes();
				if (attributes.hasAttribute("filename")){
					QString maskpath = attributes.value("filename").toString();
					objectMasks.push_back(cv::imread(roomFolder+"/"+(maskpath.toStdString().c_str()), CV_LOAD_IMAGE_UNCHANGED));
				}else{break;}

				if (attributes.hasAttribute("image_number")){
					QString depthpath = attributes.value("image_number").toString();
					number = atoi(depthpath.toStdString().c_str());
					imgNumber.push_back(number);
				}else{break;}
			}
		}
	}
}

void getQuasimodoObjects(std::string path, std::vector< std::vector<cv::Mat > > & all_objectMasks, std::vector< std::vector<unsigned int > > & all_imgNumber, std::vector<std::string > & all_names){
	std::string roomFolder = getRoomFolder(path);
	QStringList objectFiles = QDir(roomFolder.c_str()).entryList(QStringList("dynamic_obj*.xml"));
	if(objectFiles.size() == 0){return;}

	for (auto objectFile : objectFiles){
		std::string object = roomFolder+"/"+objectFile.toStdString();
		//printf("object: %s\n",object.c_str());

		std::vector<cv::Mat > objectMasks;
		std::vector<unsigned int > imgNumber;

		QFile objfile(object.c_str());

		if (!objfile.exists()){
			ROS_ERROR("Could not open file %s to masks.",object.c_str());
			continue;
		}
		getQuasimodoObject(object,objectMasks,imgNumber);

//        objfile.open(QIODevice::ReadOnly);
//        ROS_INFO_STREAM("Parsing xml file: "<<object.c_str());

//        QXmlStreamReader* objxmlReader = new QXmlStreamReader(&objfile);

//        while (!objxmlReader->atEnd() && !objxmlReader->hasError()){
//            QXmlStreamReader::TokenType token = objxmlReader->readNext();
//            if (token == QXmlStreamReader::StartDocument)
//                continue;

//            if (objxmlReader->hasError()){
//                ROS_ERROR("XML error: %s in %s",objxmlReader->errorString().toStdString().c_str(),object.c_str());
//                break;
//            }

//            QString elementName = objxmlReader->name().toString();

//            if (token == QXmlStreamReader::StartElement){
//                if (objxmlReader->name() == "Mask"){
//                    int number = 0;
//                    QXmlStreamAttributes attributes = objxmlReader->attributes();
//                    if (attributes.hasAttribute("filename")){
//                        QString maskpath = attributes.value("filename").toString();
//                        objectMasks.push_back(cv::imread(roomFolder+"/"+(maskpath.toStdString().c_str()), CV_LOAD_IMAGE_UNCHANGED));
//                    }else{break;}

//                    if (attributes.hasAttribute("image_number")){
//                        QString depthpath = attributes.value("image_number").toString();
//                        number = atoi(depthpath.toStdString().c_str());
//                        imgNumber.push_back(number);
//                    }else{break;}
//                }
//            }
//        }
		all_objectMasks.push_back(objectMasks);
		all_imgNumber.push_back(imgNumber);
		all_names.push_back(roomFolder+"/"+objectFile.toStdString());
	}
}

void getRaresObject(std::string object, std::vector<cv::Mat > & objectMasks, std::vector<unsigned int > & imgNumber){

	std::string roomFolder = getRoomFolder(object);

	QFile objfile(object.c_str());

	if (!objfile.exists()){
		ROS_ERROR("Could not open file %s to masks.",object.c_str());
		return;
	}

	objfile.open(QIODevice::ReadOnly);
	//ROS_INFO_STREAM("Parsing xml file: "<<object.c_str());

	QXmlStreamReader* objxmlReader = new QXmlStreamReader(&objfile);

	while (!objxmlReader->atEnd() && !objxmlReader->hasError()){
		QXmlStreamReader::TokenType token = objxmlReader->readNext();
		if (token == QXmlStreamReader::StartDocument)
			continue;

		if (objxmlReader->hasError()){
			ROS_ERROR("XML error: %s in %s",objxmlReader->errorString().toStdString().c_str(),object.c_str());
			break;
		}

		QString elementName = objxmlReader->name().toString();

		if (token == QXmlStreamReader::StartElement){
			if (objxmlReader->name() == "Mask"){
				int number = 0;
				QXmlStreamAttributes attributes = objxmlReader->attributes();
				if (attributes.hasAttribute("filename")){
					QString maskpath = attributes.value("filename").toString();
					//printf("path...: %s\n",(roomFolder+"/"+(maskpath.toStdString().c_str())).c_str());

					cv::Mat mask = cv::imread(roomFolder+"/"+(maskpath.toStdString().c_str()), CV_LOAD_IMAGE_UNCHANGED);
					cv::Mat img;
					img.create(480,640,CV_8UC1);

					unsigned char * maskdata = mask.data;
					unsigned char * imgdata	 = img.data;
					for(unsigned int i = 0; i < 640*480; i++){
						if(maskdata[3*i+0] == 0 && maskdata[3*i+1] == 0 && maskdata[3*i+2] == 0){
							imgdata[i] = 0;
						}else{
							imgdata[i] = 255;
						}
					}
					objectMasks.push_back(img);

					//cv::namedWindow( "Mask", cv::WINDOW_AUTOSIZE );
					//cv::imshow( "Mask",objectMasks.back() );
					//char c = cv::waitKey(0);
				}else{break;}

				if (attributes.hasAttribute("image_number")){
					QString depthpath = attributes.value("image_number").toString();
					number = atoi(depthpath.toStdString().c_str());
					imgNumber.push_back(number);
				}else{break;}
			}
		}
	}
}

void getRaresObjects(std::string path, std::vector< std::vector<cv::Mat > > & all_objectMasks, std::vector< std::vector<unsigned int > > & all_imgNumber, std::vector<std::string > & all_names){

	std::string roomFolder = getRoomFolder(path);
	roomFolder += "/mr_clusters/";
	//printf("getRaresObjects: %s\n",roomFolder.c_str());
	QStringList objectFiles = QDir(roomFolder.c_str()).entryList(QStringList("dynamic_obj*.xml"));

	for (auto objectFile : objectFiles){
		std::string object = roomFolder+"/"+objectFile.toStdString();
		//printf("object: %s\n",object.c_str());

		std::vector<cv::Mat > objectMasks;
		std::vector<unsigned int > imgNumber;

		QFile objfile(object.c_str());

		if (!objfile.exists()){
			ROS_ERROR("Could not open file %s to masks.",object.c_str());
			continue;
		}

		getRaresObject(object,objectMasks,imgNumber);

//        objfile.open(QIODevice::ReadOnly);
//        ROS_INFO_STREAM("Parsing xml file: "<<object.c_str());

//        QXmlStreamReader* objxmlReader = new QXmlStreamReader(&objfile);

//        while (!objxmlReader->atEnd() && !objxmlReader->hasError()){
//            QXmlStreamReader::TokenType token = objxmlReader->readNext();
//            if (token == QXmlStreamReader::StartDocument)
//                continue;

//            if (objxmlReader->hasError()){
//                ROS_ERROR("XML error: %s in %s",objxmlReader->errorString().toStdString().c_str(),object.c_str());
//                break;
//            }

//            QString elementName = objxmlReader->name().toString();

//            if (token == QXmlStreamReader::StartElement){
//                if (objxmlReader->name() == "Mask"){
//                    int number = 0;
//                    QXmlStreamAttributes attributes = objxmlReader->attributes();
//                    if (attributes.hasAttribute("filename")){
//                        QString maskpath = attributes.value("filename").toString();
//                        //printf("path...: %s\n",(roomFolder+"/"+(maskpath.toStdString().c_str())).c_str());

//                        cv::Mat mask = cv::imread(roomFolder+"/"+(maskpath.toStdString().c_str()), CV_LOAD_IMAGE_UNCHANGED);
//                        cv::Mat img;
//                        img.create(480,640,CV_8UC1);

//                        unsigned char * maskdata = mask.data;
//                        unsigned char * imgdata	 = img.data;
//                        for(unsigned int i = 0; i < 640*480; i++){
//                            if(maskdata[3*i+0] == 0 && maskdata[3*i+1] == 0 && maskdata[3*i+2] == 0){
//                                imgdata[i] = 0;
//                            }else{
//                                imgdata[i] = 255;
//                            }
//                        }
//                        objectMasks.push_back(img);

//                        //cv::namedWindow( "Mask", cv::WINDOW_AUTOSIZE );
//                        //cv::imshow( "Mask",objectMasks.back() );
//                        //char c = cv::waitKey(0);
//                    }else{break;}

//                    if (attributes.hasAttribute("image_number")){
//                        QString depthpath = attributes.value("image_number").toString();
//                        number = atoi(depthpath.toStdString().c_str());
//                        imgNumber.push_back(number);
//                    }else{break;}
//                }
//            }
//        }
		all_objectMasks.push_back(objectMasks);
		all_imgNumber.push_back(imgNumber);
		all_names.push_back(roomFolder+"/"+objectFile.toStdString());
	}
}


void getMetaroomObjects(std::string path, std::vector< std::vector<cv::Mat > > & all_objectMasks, std::vector< std::vector<unsigned int > > & all_imgNumber, std::vector<std::string > & all_names){
	auto objects = semantic_map_load_utilties::loadAllDynamicObjectsFromSingleSweep<PointType>(path,true);
	for (auto object : objects){
		std::cout <<"Object has "<<object.vAdditionalViews.size()<<" additional clouds "<<std::endl;
		/*
			if (object.intermediateCloud->points.size()){

				ROS_INFO_STREAM("The object contains "<<object.vAdditionalViewMaskImages.size()<<" additional image masks");
				for (size_t i=0; i<object.vAdditionalViewMaskImages.size(); i++){
					ROS_INFO_STREAM("Additional image mask "<<i<<" number of indices "<<object.vAdditionalViewMaskIndices[i].size());
					cv::imshow( "Display window", object.vAdditionalViewMaskImages[i] );
					cv::waitKey(0);

				}
			} else {
				std::cout<<"Intermediate cloud hasn't been set for this dynamic object!!!"<<std::endl;
			}
		}
		*/
	}
}

int annotateObject(std::vector< cv::Mat > rgbs, std::vector<cv::Mat> depths, std::vector<cv::Mat > objectMasks, std::vector<unsigned int > imgNumber){
	int anno = 0;

	int max_pixel = 0;
	int current_displayInd = 0;
	for(unsigned int i = 0; i < objectMasks.size(); i++){
		unsigned char * data = objectMasks[i].data;
		unsigned int pixels = objectMasks[i].rows * objectMasks[i].cols;
		unsigned int nrp = 0;
		for(unsigned int p = 0; p < pixels; p++){
			nrp +=  data[p] != 0;
		}
		if(nrp > max_pixel){
			max_pixel = nrp;
			current_displayInd = i;
		}
	}

	int fontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
	double fontScale = 1;
	int thickness = 1;
	int state = 0;

	cv::Mat rgb		= rgbs[imgNumber[current_displayInd]].clone();
	cv::Mat depth	= depths[imgNumber[current_displayInd]].clone();
	cv::Mat mask	= objectMasks[current_displayInd].clone();
	while(true){
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		cv::findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		for( unsigned int i = 0; i < contours.size(); i++ ){
			cv::drawContours( rgb, contours, i, cv::Scalar( 0, 0, 255 ), 2, 8, hierarchy, 0, cv::Point() );
			cv::drawContours( rgb, contours, i, cv::Scalar( 0, 255, 0 ), 1, 8, hierarchy, 0, cv::Point() );
		}

		unsigned int height	= rgb.rows;
		unsigned int width	= rgb.cols;
		unsigned int newwidth = width+600;

		unsigned char * rgbdata = rgb.data;
		unsigned short * depthdata = (unsigned short * )(depth.data);

		cv::Mat img;
		img.create(height,newwidth,CV_8UC3);
		unsigned char * imgdata = img.data;

		for(unsigned int i = 0; i < 3*height*newwidth; i++){imgdata[i] = 0;}

		for(unsigned int w = 0; w < width;w++){
			for(unsigned int h = 0; h < height;h++){
				int oind = h*width+w;
				int nind = h*newwidth+w;
				imgdata[3*nind + 0] = rgbdata[3*oind + 0];
				imgdata[3*nind + 1] = rgbdata[3*oind + 1];
				imgdata[3*nind + 2] = rgbdata[3*oind + 2];
				if(depthdata[oind] == 0){
					imgdata[3*nind + 0] = 255;
					imgdata[3*nind + 1] = 0;
					imgdata[3*nind + 2] = 255;
				}
			}
		}

		int textnr = 0;
//        putText(img, "1: correct                   ",       cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
//        putText(img, "2: junk                      ",       cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
//        putText(img, "3: undersegmented (correct+junk",     cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
//        putText(img, "4: undersegmented (correct+correct)", cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
//        putText(img, "5: severly oversegmented",            cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
//        putText(img, "6: unknown",                          cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
//        putText(img, "next: W previous: Q",                 cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

		putText(img, "1: correct (overlap >  90%)   ",       cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
		putText(img, "2: correct (overlap >= 50%)   ",       cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
		putText(img, "3: correct (overlap <  50%)   ",       cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
		putText(img, "4: undersegmented            ",       cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
		putText(img, "5: includes junk             ",       cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
		putText(img, "6: unknown                   ",       cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
		putText(img, "next: W previous: Q",                 cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);


		char buf [1024];
		sprintf(buf,"   Image:     %i / %i",current_displayInd+1,objectMasks.size());
		putText(img, std::string(buf), cv::Point(width+10,			30+(textnr++)*25   ), fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

//        putText(img, "   Select state(pushed CTRL)",cv::Point(width+10,	30+(textnr++)*25   ),	fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
//        putText(img, "--", cv::Point(width+5,	30+state*25   ), fontFace, fontScale, cv::Scalar(0,255,0), thickness, 8);

		cv::namedWindow( "Annotation tool",				cv::WINDOW_AUTOSIZE );	cv::imshow( "Annotation tool",img );
		char c = cv::waitKey(0);
		if(c == 'q' || c == 'Q'){
			current_displayInd = std::max(int(current_displayInd-1),0);
			rgb = rgbs[imgNumber[current_displayInd]].clone();
			depth = depths[imgNumber[current_displayInd]].clone();
			mask = objectMasks[current_displayInd].clone();
		}

		if(c == 'w' || c == 'W'){
			current_displayInd = std::min(int(current_displayInd+1),int(imgNumber.size()-1));
			rgb = rgbs[imgNumber[current_displayInd]].clone();
			depth = depths[imgNumber[current_displayInd]].clone();
			mask = objectMasks[current_displayInd].clone();
		}

		if(c == '1'){return 1;}
		if(c == '2'){return 2;}
		if(c == '3'){return 3;}
		if(c == '4'){return 4;}
		if(c == '5'){return 5;}
		if(c == '6'){return 6;}
	}

	return anno;
}

std::vector<int> annotateObjects(std::vector< cv::Mat > rgbs, std::vector<cv::Mat> depths, std::vector< std::vector<cv::Mat > > & all_objectMasks, std::vector< std::vector<unsigned int > > & all_imgNumber){
	std::vector<int> annos;
	for(unsigned int i = 0; i < all_objectMasks.size(); i++){
		annos.push_back(annotateObject(rgbs,depths,all_objectMasks[i], all_imgNumber[i]));
	}
	return annos;
}

void saveAnnotationResults(std::string filename, std::vector<int> labels,  std::vector<std::string > names){
	std::string total = "";
	for(unsigned int i = 0; i < labels.size(); i++){
		total += std::to_string(labels[i])+" "+names[i]+"\n";
	}
	//std::cout << total << std::endl;

	std::ofstream myfile;
	myfile.open (filename);
	myfile << total;
	myfile.close();

}


bool annotate(std::string path){
	//printf("annotate: %s\n",path.c_str());
	std::string roomFolder = getRoomFolder(path);

	std::vector<cv::Mat> rgbs	=  getRGBvec(path);
	std::vector<cv::Mat> depths =  getDvec(path);
	QFile raresfile((roomFolder+"/rares_annotation3.txt").c_str());
	if (!raresfile.exists()){
		printf("doing rares segments\n");
		std::vector< std::vector<cv::Mat > > quasuimodo_all_objectMasks;
		std::vector< std::vector<unsigned int > > quasuimodo_all_imgNumber;
		std::vector<std::string > quasuimodo_all_names;
		getRaresObjects(path,quasuimodo_all_objectMasks,quasuimodo_all_imgNumber,quasuimodo_all_names);

		//drawSeg(rgbs,quasuimodo_all_objectMasks,quasuimodo_all_imgNumber);
		std::vector<int> quasimodo_labels = annotateObjects(rgbs,depths,quasuimodo_all_objectMasks,quasuimodo_all_imgNumber);
		saveAnnotationResults(roomFolder+"/rares_annotation3.txt", quasimodo_labels, quasuimodo_all_names);
	}

	QFile quasimodofile((roomFolder+"/quasimodo_annotation3.txt").c_str());
	if (!quasimodofile.exists()){
		printf("doing quasimodo segments\n");
		std::vector< std::vector<cv::Mat > > quasuimodo_all_objectMasks;
		std::vector< std::vector<unsigned int > > quasuimodo_all_imgNumber;
		std::vector<std::string > quasuimodo_all_names;
		getQuasimodoObjects(path,quasuimodo_all_objectMasks,quasuimodo_all_imgNumber,quasuimodo_all_names);

		std::vector<int> quasimodo_labels = annotateObjects(rgbs,depths,quasuimodo_all_objectMasks,quasuimodo_all_imgNumber);
		saveAnnotationResults(roomFolder+"/quasimodo_annotation3.txt", quasimodo_labels, quasuimodo_all_names);
	}

	return true;
}

std::vector<int> getInds(std::string line){
	std::vector<int> ret;
	std::string current = "";
	for(unsigned int i = 0; i < line.length(); i++){
		char c = line[i];
		if(c == ' '){
			ret.push_back(atoi(current.c_str()));
			current = "";
		}else if( c >= '0' && c <= '9'){
			current += c;
		}
	}
	return ret;
}

int annotateMergeQuasimodo(std::string root, std::string folder, std::vector< int > labels, std::vector< std::string > object_paths, std::vector<int> inds){
	if(inds.size() == 1){
		return labels[inds[0]];
	}


	for(unsigned int i = 0; i < inds.size(); i++){//If anything is junk, everything is junk
		if(labels[inds[i]] == 4){return 4;}
	}


	for(unsigned int i = 0; i < inds.size(); i++){//If anything is undersegmented, everything is undersegmented
		if(labels[inds[i]] == 3){return 4;}
	}

	unsigned int unknown_count = 0;
	for(unsigned int i = 0; i < inds.size(); i++){//If everything is unknown, everything is unknown...
		if(labels[inds[i]] == 5){unknown_count++;}
	}
	if(unknown_count == inds.size()){return 5;}

	printf("---------------------------------------------\n");
	for(unsigned int i = 0; i < inds.size(); i++){
		printf("inds: %i -> label: %i -> path: %s\n",inds[i],labels[inds[i]],object_paths[inds[i]].c_str());
	}

	std::vector<cv::Mat > full_objectMasks;
	std::vector<unsigned int > full_imgNumber;

	std::vector<cv::Mat> rgbs	=  getRGBvec(folder+"/");
	std::vector<cv::Mat> depths =  getDvec(folder+"/");

	for(unsigned int i = 0; i < inds.size(); i++){
		std::vector<cv::Mat > objectMasks;
		std::vector<unsigned int > imgNumber;
		getQuasimodoObject(object_paths[inds[i]],objectMasks,imgNumber);

		for(unsigned int j = 0; j < imgNumber.size(); j++){
			bool is_new = true;
			for(unsigned int k = 0; k < full_imgNumber.size(); k++){
				if(imgNumber[j] == full_imgNumber[k]){//found this img before, merge
					is_new = false;

					unsigned char * data_old = full_objectMasks[k].data;
					unsigned char * data_new =      objectMasks[j].data;

					for(unsigned int n = 0; n < 640*480; n++){
						data_old[n] = std::max(data_old[n],data_new[n]);
					}
				}
			}
			if(is_new){
				full_objectMasks.push_back(objectMasks[j]);
				full_imgNumber.push_back(imgNumber[j]);
			}
		}
	}



	std::vector< std::vector<cv::Mat > > all_objectMasks;
	std::vector< std::vector<unsigned int > > all_imgNumber;
	all_objectMasks.push_back(full_objectMasks);
	all_imgNumber.push_back(full_imgNumber);

	std::vector<int> final_labels = annotateObjects(rgbs,depths,all_objectMasks,all_imgNumber);

	printf("---------------------------------------------\n");

	return final_labels[0];
}

bool annotateMerges(std::string root, std::string path){

	printf("annotateMerge(%s)\n",path.c_str());
	std::string roomFolder = getRoomFolder(path);

	std::string quasimodofile_path = roomFolder+"/quasimodo_annotation3.txt";
	QFile quasimodofile(quasimodofile_path.c_str());
	if (quasimodofile.exists()){
		std::string consolidate_file_path = roomFolder+"/consolidated_indices.txt";
		QFile consolidate_file(consolidate_file_path.c_str());
		if (consolidate_file.exists()){
			printf("consolidate_file: %s\n",consolidate_file_path.c_str());
			std::string line;

			std::vector< int > labels;
			std::vector< std::string > object_paths;

			std::ifstream label_file (quasimodofile_path);
			if (label_file.is_open()){
				while ( getline (label_file,line) ){
					object_paths.push_back(root+"/"+line.substr(2,line.length()-2));
					labels.push_back(int(line.front())-int('1'));
				}
				label_file.close();
			} else {std::cout << "Unable to open file";}


			std::ifstream myfile (consolidate_file_path);
			if (myfile.is_open()){
				while ( getline (myfile,line) ){
					int nr = annotateMergeQuasimodo(root,roomFolder,labels,object_paths,getInds(line));
				}
				myfile.close();
			} else {std::cout << "Unable to open file";}
		}else{
			printf("consolidate_file does not exist\n");
		}
//        printf("doing quasimodo segments\n");
//        std::vector< std::vector<cv::Mat > > quasuimodo_all_objectMasks;
//        std::vector< std::vector<unsigned int > > quasuimodo_all_imgNumber;
//        std::vector<std::string > quasuimodo_all_names;
//        getQuasimodoObjects(path,quasuimodo_all_objectMasks,quasuimodo_all_imgNumber,quasuimodo_all_names);

//        std::vector<int> quasimodo_labels = annotateObjects(rgbs,depths,quasuimodo_all_objectMasks,quasuimodo_all_imgNumber);
//        saveAnnotationResults(roomFolder+"/quasimodo_annotation3.txt", quasimodo_labels, quasuimodo_all_names);
	}

	return true;
}


class AnnotationResult{
	public:

	std::vector<int> data;
	std::string name;

	void print(){
		printf("\n\n");
		printf("===== algorithm: %s =====\n",name.c_str());
		double sum = data[0]+data[1]+data[2]+data[3]+data[4];
		printf("correct (overlap >  90%)          %4.4i (%5.5f %%)\n",data[0],100.0*double(data[0])/sum);
		printf("correct (overlap >= 50%)          %4.4i (%5.5f %%)\n",data[1],100.0*double(data[1])/sum);
		printf("correct (overlap <  50%)          %4.4i (%5.5f %%)\n",data[2],100.0*double(data[2])/sum);
		printf("undersegmented                    %4.4i (%5.5f %%)\n",data[3],100.0*double(data[3])/sum);
		printf("includes junk                     %4.4i (%5.5f %%)\n",data[4],100.0*double(data[4])/sum);
		printf("unknown                           %4.4i\n",data[5]);
		printf("not unknown                       %4.4i\n",int(sum));
	}

	void add(AnnotationResult a){
		data[0] += a.data[0];
		data[1] += a.data[1];
		data[2] += a.data[2];
		data[3] += a.data[3];
		data[4] += a.data[4];
		data[5] += a.data[5];
	}

	AnnotationResult(std::string name_ = ""){
		name = name_;
		data.resize(6);
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		data[4] = 0;
		data[5] = 0;
	}

	~AnnotationResult(){}
};

std::vector< cv::Mat > getPeopleDetections(std::string peoplepath){
	std::vector<std::vector<std::tuple<float, float, float, float> > > detections;
	detections = metaroom_detections::detections_for_xml(peoplepath, "intermediate_deep_detection");

	std::vector<cv::Mat> peopleMasks;
	for(unsigned int i = 0; i < detections.size(); i++){
		cv::Mat peoplemask;
		int width	= 640;
		int height	= 480;
		peoplemask.create(height,width,CV_8UC1);
		unsigned int nr_pixels = width*height;
		for(unsigned int j = 0; j < nr_pixels; j++){peoplemask.data[j] = 0;}
		peopleMasks.push_back(peoplemask);
	}

	unsigned int count = 0;
	for (std::vector<std::tuple<float, float, float, float> >& image_dets : detections) {
		for (std::tuple<float, float, float, float>& det : image_dets) {
			cv::rectangle(peopleMasks[count], cv::Rect(std::get<0>(det),std::get<1>(det),std::get<2>(det),std::get<3>(det)), cv::Scalar(255,255,255), -1);
		}
		count++;
	}

	return peopleMasks;
}

bool verifyObject_quasimodo(std::string object, std::vector< cv::Mat > rejections, double threshold = 0){
	std::vector<cv::Mat > objectMasks;
	std::vector<unsigned int > imgNumber;
	getQuasimodoObject(object,objectMasks,imgNumber);
	double sum_obj = 0;
	double sum_olp = 0;
	for(unsigned int i = 0; i < imgNumber.size(); i++){
		unsigned char * rejectionmask = rejections[imgNumber[i]].data;
		unsigned char * objectmask = objectMasks[i].data;
		int width	= 640;
		int height	= 480;
		unsigned int nr_pixels = width*height;
		for(unsigned int j = 0; j < nr_pixels; j++){
			if(objectmask[j] != 0){
				sum_obj++;
				if(rejectionmask[j] != 0){
					sum_olp++;
				}
			}
		}
	}
	return sum_olp/sum_obj <= threshold;
}

bool verifyObject_rares(std::string object, std::vector< cv::Mat > rejections, double threshold = 0){
	std::vector<cv::Mat > objectMasks;
	std::vector<unsigned int > imgNumber;
	getRaresObject(object,objectMasks,imgNumber);
	double sum_obj = 0;
	double sum_olp = 0;
	for(unsigned int i = 0; i < imgNumber.size(); i++){
		unsigned char * rejectionmask = rejections[imgNumber[i]].data;
		unsigned char * objectmask = objectMasks[i].data;
		int width	= 640;
		int height	= 480;
		unsigned int nr_pixels = width*height;
		for(unsigned int j = 0; j < nr_pixels; j++){
			if(objectmask[j] != 0){
				sum_obj++;
				if(rejectionmask[j] != 0){
					sum_olp++;
				}
			}
		}
	}
	return sum_olp/sum_obj <= threshold;
}

AnnotationResult summarize(std::string rootpath, std::string path, std::string algorithm, std::string peoplepath = "", double threshold = 0){
	AnnotationResult a = AnnotationResult();
	std::string filename = getRoomFolder(path)+"/"+algorithm+"_annotation3.txt";
	std::string line;
	std::ifstream myfile (filename);

	std::vector< cv::Mat > detmats;
	if(peoplepath.length() > 0){detmats = getPeopleDetections(peoplepath);}

	if (myfile.is_open()){
		while ( getline (myfile,line) ){
			std::string path = line.substr(2,line.length()-2);
			bool valid = true;
			if(detmats.size() > 0){
				if(algorithm.compare("quasimodo") == 0){
					valid = verifyObject_quasimodo(rootpath+"/"+path,detmats,threshold);
				}
				if(algorithm.compare("rares") == 0){
					valid = verifyObject_rares(rootpath+"/"+path,detmats,threshold);
				}
			}

			if(valid){
				a.data[int(line.front())-int('1')]++;
			}
		}
		myfile.close();
	} else {std::cout << "Unable to open file";}
	return a;
}

bool summarizeFiles(std::string path, std::string peoplepath = ""){
	vector<string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(path);

	AnnotationResult rares			  ("rares");
	AnnotationResult quasimodo		  ("quasimodo");

	for (auto sweep_xml : sweep_xmls) {
		rares.add(summarize(path,sweep_xml, "rares"));
		quasimodo.add(summarize(path,sweep_xml, "quasimodo"));
	}



	printf("\n\n\n\n");
	printf("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n");
	printf("dataset: %s\n",path.c_str());
	printf("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n");
	rares.print();
	quasimodo.print();


	for(double threshold = 0; threshold <= 0; threshold += 0.05){
		AnnotationResult rares_people     ("rares + people rejection");
		AnnotationResult quasimodo_people ("quasimodo + people rejection");
		int pl = path.length();
		int ppl = peoplepath.length();
		for (auto sweep_xml : sweep_xmls) {
			if(ppl > 0){
				std::string local = sweep_xml.substr(pl);
				local = local.substr(0,local.length()-8);
				std::string pp = peoplepath+local;

				rares_people.add(		summarize(path, sweep_xml, "rares",	   pp,threshold));
				quasimodo_people.add(	summarize(path, sweep_xml, "quasimodo",pp,threshold));
			}
		}

		rares_people.print();
		quasimodo_people.print();
	}
	return false;
}

bool annotateFiles(std::string path){
	//AnnotationResult rares     ("rares");
	//AnnotationResult quasimodo ("quasimodo");

	vector<string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(path);
	//for (auto sweep_xml : sweep_xmls) {
	for (unsigned int i = 0; i < sweep_xmls.size(); i++) {
		//printf("\n\n\n\n");
		//printf("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n");
		//printf("sweep %i / %i\n",i+1,sweep_xmls.size());

		annotate(sweep_xmls[i]);
		annotateMerges(path,sweep_xmls[i]);

		//rares.add(summarize(sweep_xmls[i], "rares"));
		//quasimodo.add(summarize(sweep_xmls[i], "quasimodo"));
		//rares.print();
		//quasimodo.print();

	}
	return false;
}

int main(int argc, char** argv){

	visualization_lvl = 0;

	std::vector< std::string > peoplepaths;
	std::vector< std::string > folders;
	int inputstate = 2;
	for(int i = 1; i < argc;i++){
		printf("input: %s\n",argv[i]);
		if(std::string(argv[i]).compare("-v") == 0){			inputstate	= 1;}
		else if(std::string(argv[i]).compare("-folder") == 0){	inputstate	= 2;}
		else if(std::string(argv[i]).compare("-folders") == 0){	inputstate	= 2;}
		else if(std::string(argv[i]).compare("-file") == 0){	inputstate	= 2;}
		else if(std::string(argv[i]).compare("-files") == 0){	inputstate	= 2;}
		else if(std::string(argv[i]).compare("-people") == 0){	inputstate	= 3;}
		else if(inputstate == 1){visualization_lvl = atoi(argv[i]);}
		else if(inputstate == 2){folders.push_back(std::string(argv[i]));}
		else if(inputstate == 3){peoplepaths.push_back(std::string(argv[i]));}
	}

	if(visualization_lvl > 0){
		viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0.5, 0, 0.5);
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
	}

	if(folders.size() == 0){
		folders.push_back(std::string(getenv ("HOME"))+"/.semanticMap/");
		peoplepaths.push_back(folders.back());
	}

	//for(unsigned int i = 0; i < folders.size(); i++){annotateFiles(folders[i]);}
	for(unsigned int i = 0; i < folders.size(); i++){summarizeFiles(folders[i],peoplepaths[i]);}

	printf("done annotating\n");

	return 0;
}
