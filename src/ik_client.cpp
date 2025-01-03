#include <ros/ros.h>
#include "ik_service/PoseIK.h" // The custom service definition
#include "geometry_msgs/Pose.h" // Pose message type

int main(int argc, char **argv) {
    ros::init(argc, argv, "ik_client");
    ros::NodeHandle nh;

    // Create a client for the IK service
    ros::ServiceClient client = nh.serviceClient<ik_service::PoseIK>("pose_ik");

    // Create a request
    ik_service::PoseIK srv;
    // Declare a Pose variable
    geometry_msgs::Pose part_pose;

    srv.request.part_pose.position.x = 0.5;
    srv.request.part_pose.position.y = 0.0;
    srv.request.part_pose.position.z = 0.5;
    srv.request.part_pose.orientation.x = 0.0;
    srv.request.part_pose.orientation.y = 0.0;
    srv.request.part_pose.orientation.z = 0.0;
    srv.request.part_pose.orientation.w = 1.0;

//    srv.request = part_pose;

    if (client.call(srv)) {
        ROS_INFO("Call to ik_service returned [%i] solutions", srv.response.num_sols);
        for (int i = 0; i < srv.response.num_sols; ++i) {
            ROS_INFO("Solution %d: [%f, %f, %f, %f, %f, %f]",
                     i,
                     srv.response.joint_solutions[i].joint_angles[0],
                     srv.response.joint_solutions[i].joint_angles[1],
                     srv.response.joint_solutions[i].joint_angles[2],
                     srv.response.joint_solutions[i].joint_angles[3],
                     srv.response.joint_solutions[i].joint_angles[4],
                     srv.response.joint_solutions[i].joint_angles[5]);
        }
    } else {
        ROS_ERROR("Failed to call service ik_service");
    }

    return 0;
}
