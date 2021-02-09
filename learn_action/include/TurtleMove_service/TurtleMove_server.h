#ifndef _TURTLEMOVE_SERVER_H_
#define _TURTLEMOVE_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "learn_action/TurtleMoveAction.h"
#include <boost/bind.hpp>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <airface_drive_msgs/getpose.h>

namespace RobotArm{

    typedef actionlib::SimpleActionServer<learn_action::TurtleMoveAction> Server;

    class Move{
    public:
        Move();
        ~Move();
        void posecallback(const turtlesim::PoseConstPtr& msg);
        void executeCb(const learn_action::TurtleMoveGoalConstPtr& goal, Server* as);

    private:
        ros::NodeHandle nh;
        ros::Publisher turtle_vel;
        ros::Subscriber turtle_node;
        ros::ServiceClient pose;
        Server *server;
    };

    struct Myturtle
    {
        float x;
        float y;
        float theta;
    }turtle_original_pose,turtle_target_pose;
}


#endif
