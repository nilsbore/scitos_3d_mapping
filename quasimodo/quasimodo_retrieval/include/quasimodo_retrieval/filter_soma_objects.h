#ifndef FILTER_SOMA_OBJECTS_H
#define FILTER_SOMA_OBJECTS_H

#include <ros/ros.h>
#include <quasimodo_msgs/query_cloud.h>

namespace quasimodo_retrieval {

void filter_soma_objects(ros::NodeHandle& n, quasimodo_msgs::query_cloud::Response& raw_res,
                         quasimodo_msgs::query_cloud::Response& res);

} // namespace quasimodo_retrieval

#endif // FILTER_SOMA_OBJECTS_H
