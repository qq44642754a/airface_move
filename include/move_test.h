#ifndef _MOVE_TEST_H_
#define _MOVE_TEST_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <airface_drive_msgs/getpose.h>

using namespace std;

class MoveTest
{
private:
    move_base_msgs::MoveBaseGoal goal;
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped Marker_pose, Marker_pose_tmp;
    ros::ServiceClient pose;
    int get_pose, robot_move, getflagsuccess ;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base;
public:
    MoveTest();
    ~MoveTest();
    int move_finish;
    void transform_tf();
    void goSP(); // go to the startpoint
    void initMove();
    void poseCallback(const visualization_msgs::Marker &marker_tmp);
};

#endif
