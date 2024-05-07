#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include "geometry_msgs/PoseStamped.h"
#include "Tools/InitParam.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "position_publisher");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/morph_quad_x/command/pose", 10);
    ros::Rate loop_rate(500);

    while (ros::ok())
    {
	iniLoad("/home/lanny/rotors_ws/src/morph_simulator/morph_control/src/nodes/position.ini");
    double goal_position_x = iniGet<double>("goal_position_x", 0.0);
    double goal_position_y = iniGet<double>("goal_position_y", 0.0);
    double goal_position_z = iniGet<double>("goal_position_z", 3.0);
    ROS_INFO("get goal:%f,%f,%f",goal_position_x,goal_position_y,goal_position_z);

    geometry_msgs::PoseStamped position_msg;

    position_msg.header.seq=0;
    position_msg.header.stamp.sec=0;
    position_msg.header.stamp.nsec=0;
    position_msg.pose.position.x=goal_position_x;
    position_msg.pose.position.y=goal_position_y;
    position_msg.pose.position.z=goal_position_z;
    pub.publish(position_msg);
    loop_rate.sleep();
    }
    return 0;
}
