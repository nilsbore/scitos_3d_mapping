#ifndef OBJECT_RETRIEVAL_H
#define OBJECT_RETRIEVAL_H

#include <vocabulary_tree/vocabulary_tree.h>
#include <reweighted_vocabulary_tree/reweighted_vocabulary_tree.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

class object_retrieval
{
public:
    // using SIFT
    //static const int N = 131;

    // using SHOT
    static const int N = 1344;

    static std::string feature_vocabulary_file;
    static std::string feature_segment_file;
    static std::string indices_segment_file;

    using PointT = pcl::PointXYZRGB;
    using CloudT = pcl::PointCloud<PointT>;
    using HistT = pcl::Histogram<N>;
    using HistCloudT = pcl::PointCloud<HistT>;
    using NormalT = pcl::Normal;
    using NormalCloudT = pcl::PointCloud<NormalT>;
    using index_score = vocabulary_tree<HistT, 8>::cloud_idx_score;

    std::string segment_path;
    vocabulary_tree<HistT, 8> vt;
    reweighted_vocabulary_tree<HistT, 8> rvt;
    std::set<int> exclude_set;

    void set_exclude_set(const std::set<int>& other_set) { exclude_set = other_set; }

    void visualize_cloud(CloudT::Ptr& cloud);
    void extract_features(std::vector<int>& inds, HistCloudT::Ptr& features, std::vector<CloudT::Ptr>& segments,
                          std::vector<NormalCloudT::Ptr>& normals, std::vector<CloudT::Ptr>& hd_segments, const Eigen::Matrix3f& K);
    void extract_feature(vector<int>& inds, HistCloudT::Ptr& feature, CloudT::Ptr& segment,
                         NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment, const Eigen::Matrix3f& K, int ind);
    void get_query_cloud(HistCloudT::Ptr& query_cloud, CloudT::Ptr& segment, NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment, Eigen::Matrix3f& K);
    size_t write_segments(std::vector<CloudT::Ptr>& segments, std::vector<NormalCloudT::Ptr>& normals, std::vector<CloudT::Ptr>& hd_segments,  const Eigen::Matrix3f& K, vector<string>& files, size_t istart);
    void read_segments(std::vector<CloudT::Ptr>& segments, std::vector<NormalCloudT::Ptr>& normals, std::vector<CloudT::Ptr>& hd_segments,  Eigen::Matrix3f& K, size_t max_segments);
    bool read_segment(CloudT::Ptr& segment, NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment, Eigen::Matrix3f& K, string& metadata, size_t segment_id);
    bool read_other_segment(CloudT::Ptr& segment, NormalCloudT::Ptr& normal, CloudT::Ptr& hd_segment, Eigen::Matrix3f& K,
                            string& metadata, size_t segment_id, const std::string& other_segment_path);
    void write_vocabulary(vocabulary_tree<HistT, 8>& vt);
    void read_vocabulary(vocabulary_tree<HistT, 8>& rvt);
    float calculate_similarity(CloudT::Ptr& cloud1, const Eigen::Matrix3f& K1,
                               CloudT::Ptr& cloud2, const Eigen::Matrix3f& K2);
    size_t compute_segments(std::vector<CloudT::Ptr>& sweeps, std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >& intrinsics, vector<string>& files, size_t i = 0);
    void process_segments();
    void process_segments_incremental();
    void train_vocabulary_incremental(int max_segments, bool simply_train = false);
    void query_vocabulary(vector<index_score>& scores, size_t query_ind, size_t nbr_query, bool visualize_query = false, int number_original_features = 0, const string &other_segments_path = "");
    void query_reweight_vocabulary(vector<index_score>& first_scores, vector<index_score>& reweight_scores,
                                   size_t query_ind, size_t nbr_query, bool visualize_query,
                                   int number_original_features, const string& other_segments_path);

    void save_features(HistCloudT::Ptr& features, std::vector<int>& indices);
    bool load_features(HistCloudT::Ptr& features, std::vector<int>& indices);
    void save_features_for_segment(HistCloudT::Ptr& features, int i);
    bool load_features_for_segment(HistCloudT::Ptr& features, int i);
    bool load_features_for_other_segment(HistCloudT::Ptr& features, const std::string& other_segment_path, int i);
    int add_others_to_vocabulary(int max_segments, const std::string& other_segment_path, int nbr_original_segments = -1, bool add_some = false);

    object_retrieval(const std::string& segment_path);
};

#endif // OBJECT_RETRIEVAL_H
