#ifndef _TURTLEMOVE_CLIENT_H_
#define _TURTLEMOVE_CLIENT_H_

#include <actionlib/client/simple_action_client.h>
#include "learn_action/TurtleMoveAction.h"
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <boost/bind.hpp>
#include <iostream>
#include <airface_drive_msgs/getpose.h>

using  namespace  std;

namespace RobotArm{

    typedef actionlib::SimpleActionClient<learn_action::TurtleMoveAction> Client;
    struct Myturtle
    {
        float x;
        float y;
        float theta;
    }turtle_present_pose;

    class MoveReq{
    public:
        MoveReq();
        ~MoveReq();

        void doneCb(const actionlib::SimpleClientGoalState& state,
                    const learn_action::TurtleMoveResultConstPtr& result);

        void activeCb();

        void feedbackCb(const learn_action::TurtleMoveFeedbackConstPtr& feedback);

    private:
        ros::NodeHandle nh;
        ros::ServiceClient pose;
        Client *ac_;

    };


}



#endif
