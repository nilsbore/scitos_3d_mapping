#include <quasimodo_msgs/retrieval_query_result.h>
#include <quasimodo_msgs/retrieval_query.h>
#include <quasimodo_msgs/simple_query_cloud.h>
#include <quasimodo_msgs/query_cloud.h>
#include <quasimodo_retrieval/filter_soma_objects.h>

using namespace std;

class filtering_node {

public:

    ros::NodeHandle n;
    ros::ServiceServer service;
    string raw_service_name;

    filtering_node(const string& name)
    {
        ros::NodeHandle pn("~");
        string service_name;
        pn.param<string>("filtered_service_name", service_name, "/quasimodo_retrieval_service");
        pn.param<string>("raw_service_name", raw_service_name, "/raw_quasimodo_retrieval_service");
        service = n.advertiseService(service_name, &filtering_node::service_callback, this);
    }

    bool service_callback(quasimodo_msgs::query_cloud::Request& req,
                          quasimodo_msgs::query_cloud::Response& res)
    {
        ros::ServiceClient service = n.serviceClient<quasimodo_msgs::query_cloud>(raw_service_name);
        service.waitForExistence(ros::Duration(1.0));
        quasimodo_msgs::query_cloud::Response raw_res;
        service.call(req, raw_res);
        quasimodo_retrieval::filter_soma_objects(n, raw_res, res);
        return true;
    }

}; // class filtering_node

int main(int argc, char** argv)
{

    ros::init(argc, argv, "quasimodo_filtering_node");

    filtering_node fs(ros::this_node::getName());

    ros::spin();

    return 0;
}
