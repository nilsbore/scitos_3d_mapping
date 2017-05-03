#ifndef reglibsuperpoint_H
#define reglibsuperpoint_H

#include <Eigen/Dense>


namespace reglib
{

double mysign(double v);

class superpoint{
public:
//	Eigen::Vector3f point;
//	Eigen::Vector3f normal;
//	Eigen::VectorXf feature;
	double x;
	double y;
	double z;

	double nx;
	double ny;
	double nz;

	double r;
	double g;
	double b;

	double point_information;
	double normal_information;
	double colour_information;
	//double feature_information;
	int last_update_frame_id;
    bool is_boundry;

    superpoint(Eigen::Vector3f p = Eigen::Vector3f(0,0,0), Eigen::Vector3f n = Eigen::Vector3f(0,0,0), Eigen::VectorXf f = Eigen::VectorXf(3), double pi = 1, double fi = 1, int id = 0, bool is_boundry_ = false);
	~superpoint();
    void merge(superpoint p, double weight = 1);
    void transform(Eigen::Matrix4d cp);
    double angle(superpoint p);
    double distance(superpoint p);
    void print();

//    double residualZ = mysign(dst_z-tz)*fabs(tnx*(dst_x-tx) + tny*(dst_y-ty) + tnz*(dst_z-tz));//dst_z-tz;//mysign(dst_z-tz)*fabs(tnx*(dst_x-tx) + tny*(dst_y-ty) + tnz*(dst_z-tz));
//    double residualD2 = (dst_x-tx)*(dst_x-tx) + (dst_y-ty)*(dst_y-ty) + (dst_z-tz)*(dst_z-tz);
//    double residualR =  dst_rgbdata[3*dst_ind + 2] - sp.r;
//    double residualG =  dst_rgbdata[3*dst_ind + 1] - sp.g;
//    double residualB =  dst_rgbdata[3*dst_ind + 0] - sp.b;
//    double angle = tnx*dst_nx + tny*dst_ny + tnz*dst_nz;
//    ret.push_back(ReprojectionResult (src_ind, dst_ind, angle, residualZ,residualD2, residualR, residualG, residualB, getNoise(dst_z), 1.0));

};
}

#endif
