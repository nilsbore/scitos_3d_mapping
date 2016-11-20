#include <quasimodo_retrieval/filter_soma_objects.h>
#include <soma_manager/SOMAQueryObjs.h>

// int8  query_type    ### 0: perform soma object query                 0
//                     ### 1: return all object types and ids
//                     ### 2: return min-max timestamp limits of the collection
//
//
//
// bool uselowertime   ### use lower time limit as a query field           False
// bool useuppertime   ### use upper time limit as a query field            False
// bool usedates       ### use dates as a query field. If the upper or lower date is 0, it is ignored.   False
// bool useweekday     ### use weekday as a query field   False
// bool useroi_id      ### use roi_id as a query field. If set to true, custom_roi array will be ignored   False
//
//
// uint32 lowerhour     ### lower limit of the hour in the interval[0-23]    -
//
// uint32 upperhour     ### upper limit of the hour in the interval [0-23]   -
//
// uint32 lowerminutes  ### lower limit of minutes in the interval [0-59]   -
//
// uint32 upperminutes  ### upper limit of minutes in the interval [0-59]   -
//
// uint64 lowerdate     ### lower date in timestamp format   -
//
// uint64 upperdate     ### upper date limit in timestamp format   -
//
// uint8 weekday        ### iso weekday 1-Monday 7-Sunday   -
//
// string[] objectids   ### array of object ids that will be queried with or statement   -
//
// string[] objecttypes ### array of object types that will be queried with or statement   -
//
// string roi_id        ### id of the roi that will be used for within query   -
//
// string config        ### retrieve objects with specific configuration   -
//
// float32[] custom_roi   ### user defined custom roi with 4 vertices The element order should be -> [x_lower,x_upper,y_lower,y_upper].
//
//
// ---
// int32[]  timedatelimits    ### timedatelimits[0] -> min-date in timestamp format
//                            ### timedatelimits[1] -> max-date in timestamp format
//
// string[] types             ### vector of object types present in current collection
//
// string[] ids               ### vector of object ids present in current collection
//
// string[] unique_ids        ### vector of mongo unique ids of objects
//
// soma_msgs/SOMAObject[] objects  ### vector of returned objects
//
// string queryjson           ### performed query in json format     - WIll return the query json, to debug

namespace quasimodo_retrieval {

void filter_soma_objects(ros::NodeHandle& n, quasimodo_msgs::query_cloud::Response& raw_res,
                         quasimodo_msgs::query_cloud::Response& res)
{
    ros::ServiceClient service = n.serviceClient<soma_manager::SOMAQueryObjs>("/soma/query_objs");
    service.waitForExistence(ros::Duration(1.0));
    for (size_t i = 0; i < raw_res.retrieved_clouds.size(); ++i) {
        soma_manager::SOMAQueryObjs::Request soma_req;
        soma_req.uselowertime = false;
        soma_req.useuppertime = false;
        soma_req.usedates = false;
        soma_req.useweekday = false;
        soma_req.useroi_id = false;
        float x = raw_res.global_poses.position.x;
        float y = raw_res.global_poses.position.y;
        soma_req.custom_roi = vector<float> { x - 0.3, x + 0.3, y - 0.3, x + 0.3 };
        soma_manager::SOMAQueryObjs::Response soma_resp;
        service.call(soma_req, soma_resp);
        if (objects.size() == 0) { // keep this object in the filtered message
            res.retrieved_clouds.push_back(raw_res.retrieved_clouds[i]);
            res.retrieved_initial_poses.push_back(raw_res.retrieved_initial_poses[i]);
            res.retrieved_images.push_back(raw_res.retrieved_images[i]);
            res.retrieved_depths.push_back(raw_res.retrieved_depths[i]);
            res.retrieved_masks.push_back(raw_res.retrieved_masks[i]);
            res.retrieved_image_paths.push_back(raw_res.retrieved_image_paths[i]);
            res.retrieved_distance_scores.push_back(raw_res.retrieved_distance_scores[i]);
            res.segment_indices.push_back(raw_res.segment_indices[i]);
            res.vocabulary_ids.push_back(raw_res.vocabulary_ids[i]);
            res.global_poses.push_back(raw_res.global_poses[i]);
        }
    }
}

} // namespace quasimodo_retrieval
