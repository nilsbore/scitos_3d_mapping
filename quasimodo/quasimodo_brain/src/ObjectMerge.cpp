#include "Util/Util.h"

#include <metaroom_detections/metaroom_detections.h>

using namespace std;
using namespace pcl;
using namespace semantic_map_load_utilties;
typedef pcl::PointXYZRGB PointType;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
int visualization_lvl		= 0;

std::string getRoomFolder(std::string xmlFile){
    QString xmlFileQS(xmlFile.c_str());
    int index = xmlFileQS.lastIndexOf('/');
    return xmlFileQS.left(index).toStdString();
}

double similarity(std::vector<float> & feature1, std::vector<float> & feature2){
    double sum = 0;
    for(unsigned int i = 0; i < feature1.size(); i++){
        sum += std::min(feature1[i],feature2[i]);
    }
    return sum;
}

std::vector<float> add(std::vector< std::vector< float > > features, std::vector< PointCloud<PointXYZRGBNormal>::Ptr > clouds, std::vector< int > inds){
    return features[inds.front()];
//    double sum = 0;
//    std::vector<float> feature;
//    unsigned int dim = features.front().size();
//    feature.resize(dim);
//    for(unsigned int i = 0; i < dim; i++){feature[i] = 0;}

//    for(unsigned int k = 0; k < inds.size(); k++){
//        unsigned int ind = inds[k];
//        double nrp = clouds[k]->points.size();
//        std::vector< float > & cf = features[ind];
//        for(unsigned int i = 0; i < dim; i++){
//            feature[i] = nrp*cf[i];
//        }
//        sum += nrp;
//    }
//    for(unsigned int i = 0; i < dim; i++){feature[i] /= sum;}
//    return feature;
}

std::vector<float> getFeature(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud){
    std::vector<float> feature;
    unsigned int nrp = cloud->points.size();
    unsigned int step = std::max(int(0.5 + (nrp)/10000.0),1);

    double maxd_de  = 2.00;
    double scale_de = 0.025;
    unsigned int nr_maxind_de = int(maxd_de/scale_de)+1;

    double maxd_ne  = 2.00;
    double scale_ne = 0.05;
    unsigned int nr_maxind_ne = int(maxd_ne/scale_ne)+1;
    feature.resize(nr_maxind_de * nr_maxind_ne);

    for(unsigned int i = 0; i < feature.size(); i++){feature[i] = 0;}

    int test = 0;
    for(unsigned int i = 0; i < nrp; i+=step){
        pcl::PointXYZRGBNormal pi = cloud->points[i];
        for(unsigned int j = i+step; j < nrp; j+=step){
            pcl::PointXYZRGBNormal pj = cloud->points[j];
            double dx = pi.x-pj.x;
            double dy = pi.y-pj.y;
            double dz = pi.z-pj.z;
            int de_ind = sqrt(dx*dx+dy*dy+dz*dz)/scale_de;

            double ne  = 1 - (pi.normal_x*pj.normal_x + pi.normal_y*pj.normal_y + pi.normal_z*pj.normal_z);
            int ne_ind = ne/scale_ne;

            if(de_ind < nr_maxind_de){
                feature[de_ind * nr_maxind_ne + ne_ind]++;
            }
        }
    }
//    printf("nr_points: %6.6i -> %6.6i -> %10.10i\n",nrp,step,test);

    double sum = 0;
    for(unsigned int i = 0; i < feature.size(); i++){sum += feature[i];}
    for(unsigned int i = 0; i < feature.size(); i++){feature[i] /= sum;}

//    printf("feature: ");
//    for(unsigned int i = 0; i < feature.size(); i++){printf("%3.3f ",feature[i]);}
//    printf("\n");


    return feature;
}

void runMerge(std::string path){

    std::vector< std::vector< PointCloud<PointXYZRGBNormal>::Ptr > > all_clouds;
    std::vector< std::vector< std::vector< float > > > all_features;
    std::vector< std::vector< std::vector< int > > > all_inds;

    std::vector<std::string> sweep_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(path);
    for (unsigned int s = 0; s < sweep_xmls.size(); s++) {
        all_features.resize(all_features.size()+1);
        all_clouds.resize(all_clouds.size()+1);
        all_inds.resize(all_inds.size()+1);
        std::vector<reglib::Model *> models = quasimodo_brain::loadModels(sweep_xmls[s]);

        std::vector<double> model_meanx;
        std::vector<double> model_meany;
        std::vector<double> model_meanz;
        std::vector<double> model_size;
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        for(unsigned int m = 0; m < models.size(); m++){
            models[m]->recomputeModelPoints();
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = models[m]->getPCLnormalcloud(1,false);
//            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
//            pcl::transformPointCloud (*cloud, *tcloud, models[m]->frames.front()->pose);
//            printf("nr points: %i\n",tcloud->points.size());

            double sumx = 0;
            double sumy = 0;
            double sumz = 0;
            for(unsigned int i = 0; i < cloud->points.size(); i++){
                sumx += cloud->points[i].x;
                sumy += cloud->points[i].y;
                sumz += cloud->points[i].z;
            }

            sumx /= double(cloud->points.size());
            sumy /= double(cloud->points.size());
            sumz /= double(cloud->points.size());

            double size = 0;
            for(unsigned int i = 0; i < cloud->points.size(); i++){
                double dx = sumx - cloud->points[i].x;
                double dy = sumy - cloud->points[i].y;
                double dz = sumz - cloud->points[i].z;
                //size += sqrt(dx*dx+dy*dy+dz*dz) ;
                size = std::max(size,dx*dx+dy*dy+dz*dz);
            }

            model_meanx.push_back(sumx);
            model_meany.push_back(sumy);
            model_meanz.push_back(sumz);
            //model_size.push_back(size/double(cloud->points.size()));
            model_size.push_back(sqrt(size));

            all_clouds.back().push_back(cloud);
            all_features.back().push_back(getFeature(cloud));
            std::vector< int > inds; inds.push_back(m);
            all_inds.back().push_back(inds);

            char buf [1024];
            sprintf(buf,"cloud%i",m);
            viewer->addPointCloud<PointXYZRGBNormal> (cloud, visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal>(cloud), buf);
        }



        for(unsigned int i = 0; i < models.size(); i++){
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudi = models[i]->getPCLnormalcloud(1,false);
            PointXYZ pi;
            pi.x = model_meanx[i];
            pi.y = model_meany[i];
            pi.z = model_meanz[i];
            double si = model_size[i];
            for(unsigned int j = i+1; j < models.size(); j++){

                pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudj = models[j]->getPCLnormalcloud(1,false);
                PointXYZ pj;
                pj.x = model_meanx[j];
                pj.y = model_meany[j];
                pj.z = model_meanz[j];

                double dx = pi.x-pj.x;
                double dy = pi.y-pj.y;
                double dz = pi.z-pj.z;

                double sj = model_size[j];
                double dist = sqrt(dx*dx+dy*dy+dz*dz);
                if(dist < (si+sj)){

                    char buf [1024];
                    sprintf(buf,"line_%i_%i",i,j);
                    viewer->addLine<pcl::PointXYZ> (pi,pj,255,0,0,buf);

                    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr combinedcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
                    *combinedcloud += *cloudi;
                    *combinedcloud += *cloudj;

                    std::vector<float> feature = getFeature(combinedcloud);
                    std::vector< int > inds; inds.push_back(i); inds.push_back(j);
                    std::vector<float> sum_feature = add(all_features[s], all_clouds[s], inds);

                    std::vector<float> featurei = all_features.back()[i];
                    std::vector<float> featurej = all_features.back()[j];

                    double best = 0;
                    double best_sumsim = 0;
                    int best_k = 0;
                    int best_m = 0;
                    for(unsigned int k = 0; k < all_features.size()-1; k++){
                        for(unsigned int m = 0; m < all_features[k].size(); m++){

                            double sim  = similarity(feature,  all_features[k][m]);
                            double simi = similarity(featurei, all_features[k][m]);
                            double simj = similarity(featurej, all_features[k][m]);

                            if(sim > 0.85 && sim > (0.05+std::max(simi,simj))){
                                best = sim;
                                best_sumsim = std::max(simi,simj);
                                best_k = k;
                                best_m = m;
                            }
                        }
                    }

                    if(i > 0 && best > 0.0){

                        viewer->removeAllPointClouds();
                        viewer->removeAllShapes();

                        for(unsigned int k = 0; k < combinedcloud->points.size(); k++){
                            combinedcloud->points[k].r = 0;
                            combinedcloud->points[k].g = 255;
                            combinedcloud->points[k].b = 0;
                        }

                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr other = all_clouds[best_k][best_m];

                        for(unsigned int k = 0; k < other->points.size(); k++){
                            other->points[k].r = 255;
                            other->points[k].g = 0;
                            other->points[k].b = 0;
                        }

                        viewer->addPointCloud<PointXYZRGBNormal> (combinedcloud, visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal>(combinedcloud), "combinedcloud");
                        viewer->addPointCloud<PointXYZRGBNormal> (other, visualization::PointCloudColorHandlerRGBField<PointXYZRGBNormal>(other), "all_clouds[best_k][best_m]");
                        viewer->spin();
                    }
                    all_clouds.back().push_back(combinedcloud);
                    all_features.back().push_back(feature);
                    all_inds.back().push_back(inds);
                }
            }
        }
        //viewer->spin();


        for(unsigned int i = 0; i < models.size(); i++){
            models[i]->fullDelete();
            delete models[i];
        }
    }
}

int main(int argc, char** argv){

    visualization_lvl = 0;

    std::vector< std::string > folders;
    int inputstate = 2;
    for(int i = 1; i < argc;i++){
        printf("input: %s\n",argv[i]);
        if(std::string(argv[i]).compare("-v") == 0){                inputstate = 1;}
        else if(inputstate == 1){visualization_lvl = atoi(argv[i]); inputstate = 2; }
        else if(inputstate == 2){folders.push_back(std::string(argv[i]));}
    }

    if(visualization_lvl > 0){
        viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (1.0, 1.0, 1.0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
    }

	if(folders.size() == 0){
		folders.push_back(std::string(getenv ("HOME"))+"/.semanticMap/");
	}

    for(unsigned int i = 0; i < folders.size(); i++){
        runMerge(folders[i]);
    }

    return 0;
}
