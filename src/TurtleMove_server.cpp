#include "TurtleMove_service/TurtleMove_server.h"
#include <iostream>

using namespace std; 

namespace RobotArm{
    Move::Move(): server(NULL) {

        pose = nh.serviceClient<airface_drive_msgs::getpose>("/airface_driver/get_tf_pose");
        turtle_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

        server = new Server(nh, "TurtleMove", boost::bind(&Move::executeCb, this , _1,  server), false);
        server->start();
        ROS_INFO("server has started.");
    }

    Move::~Move() { delete server; }


//    void Move::posecallback(const turtlesim::PoseConstPtr& msg)
//    {
//        ROS_INFO("Turtle1_position:(%f,%f,%f)",msg->x,msg->y,msg->theta);
//        turtle_original_pose.x=msg->x;
//        turtle_original_pose.y=msg->y;
//        turtle_original_pose.theta=msg->theta;
//    }

    void Move::executeCb(const learn_action::TurtleMoveGoalConstPtr& goal, Server* as)
    {
        learn_action::TurtleMoveFeedback feedback;

        ROS_INFO("TurtleMove is working.");
        turtle_target_pose.x=goal->turtle_target_x;
        turtle_target_pose.y=goal->turtle_target_y;
        turtle_target_pose.theta=goal->turtle_target_theta;

        geometry_msgs::Twist vel_msgs;
        float break_flag_angle;

        ros::Rate r(10);

        while(ros::ok())
        {

            airface_drive_msgs::getpose g;
            pose.call(g);
            if(g.response.result){
                turtle_original_pose.x=g.response.x;
                turtle_original_pose.y=g.response.y;
                turtle_original_pose.theta=g.response.yaw;
            }

            vel_msgs.angular.z = 0.2;	// 定义角速度
            vel_msgs.linear.x = 0.1;	// 定义线速度  半径为 v/w
	 
	break_flag_angle = sqrt(pow(turtle_target_pose.theta - turtle_original_pose.theta,2));  // 目前角度和目标角度的绝对值

            turtle_vel.publish(vel_msgs);

            feedback.present_turtle_x=turtle_original_pose.x;
            feedback.present_turtle_y=turtle_original_pose.y;
            feedback.present_turtle_theta=turtle_original_pose.theta;
            server->publishFeedback(feedback);

            ROS_INFO("break_flag_angle=%f", break_flag_angle);
            if(break_flag_angle < 0.05 ) break;
            r.sleep();
        }
        // 当action完成后，向客户端返回结果
        ROS_INFO("TurtleMove is finished.");
        vel_msgs.angular.z = 0.0;	// action完成后，线速度角速度清0
        vel_msgs.linear.x = 0.0;
        turtle_vel.publish(vel_msgs);
        server->setSucceeded();
    }
}

 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "TurtleMove_server");

    RobotArm::Move move;

    ros::spin();
    return 0;
}

