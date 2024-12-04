#include <ros/ros.h>
#include "ik_service/PoseIK.h" // The custom service definition
#include "ur_kinematics/ur_kin.h" // Inverse kinematics library
#include "geometry_msgs/Pose.h" // Pose message type

bool poseIKCallback(ik_service::PoseIK::Request &req,
                    ik_service::PoseIK::Response &res) {
    ROS_INFO("Received IK request for pose: [x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f]",
             req.part_pose.position.x,
             req.part_pose.position.y,
             req.part_pose.position.z,
             req.part_pose.orientation.x,
             req.part_pose.orientation.y,
             req.part_pose.orientation.z,
             req.part_pose.orientation.w);

    // Define the T matrix (row-major order)
    double T[16] = {
        0.0, -1.0, 0.0, req.part_pose.position.x,
        0.0,  0.0, 1.0, req.part_pose.position.y,
       -1.0,  0.0, 0.0, req.part_pose.position.z,
        0.0,  0.0, 0.0, 1.0
    };

    ROS_INFO("Transformation Matrix (T):");
    for (int i = 0; i < 4; ++i) {
        ROS_INFO("[%f, %f, %f, %f]", T[i*4], T[i*4+1], T[i*4+2], T[i*4+3]);
    }


    // Prepare storage for solutions
    double q_sols[8][6];
    int num_sol = ur_kinematics::inverse(T, &q_sols[0][0], 0.0);
    ROS_INFO("Number of IK solutions found: %d", num_sol);

    if (num_sol > 0) {
        for (int i = 0; i < num_sol; ++i) {
            ik_service::JointSolutions joint_solution;
            for (int j = 0; j < 6; ++j) {
                joint_solution.joint_angles[j] = q_sols[i][j];
            }
            res.joint_solutions.push_back(joint_solution);
        }
    } else {
        ROS_WARN("No solutions found for this pose.");
    }

    // Fill response with the number of solutions
    res.num_sols = num_sol;

    return true; // Indicate successful execution
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ik_service");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("pose_ik", poseIKCallback);
    ROS_INFO("IK service is ready to receive requests.");

    ros::spin(); // Keep the service running
    return 0;
}
