#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

ros::Subscriber state_sub, pose_sub;

mavros_msgs::State current_state;
void MAVROSStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped iris_pose;
void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    iris_pose = *msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    // subscribe
    state_sub = nh.subscribe<mavros_msgs::State>("/iris_0/mavros/state", 1, MAVROSStateCallback);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 10, PoseCallback);

    ros::Rate rate(1000);

    ros::Time takeoff_time = ros::Time::now();
    ros::Time landing_time = ros::Time::now();
    bool isTakeoff = false;
    bool isLanding = false;

    while(ros::ok())
    {
        if (!isTakeoff && !isLanding && current_state.armed && iris_pose.pose.position.z > 0.1)
        {
            isTakeoff = true;
            takeoff_time = ros::Time::now();
            std::cout << "Start timers" << std::endl;
        }
        else if(isTakeoff && !isLanding && !current_state.armed && iris_pose.pose.position.z < 0.1)
        {
            isLanding = true;
            landing_time = ros::Time::now();
            std::cout << "End timers" << std::endl;
            std::cout << "Time Usage: " << landing_time.toSec() - takeoff_time.toSec() << " seconds" << std::endl;

            std::cout << "Distance from the center of landing area: " << std::sqrt(std::pow(iris_pose.pose.position.x - 18, 2) + std::pow(iris_pose.pose.position.y + 2, 2)) << " m" << std::endl;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}