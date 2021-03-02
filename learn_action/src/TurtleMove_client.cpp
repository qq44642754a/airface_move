#include "TurtleMove_service/TurtleMove_client.h"

 
namespace RobotArm{

    MoveReq::MoveReq(): ac_(NULL){

        ac_ = new Client("TurtleMove", true);
        ac_->waitForServer();
        pose = nh.serviceClient<airface_drive_msgs::getpose>("/airface_driver/get_tf_pose");
        ROS_INFO("Action server started, sending goal.");

        airface_drive_msgs::getpose g;
        pose.call(g);

        if(g.response.result){
 	cout << "----------------------------" << g.response.yaw << endl;
        }
        	
        learn_action::TurtleMoveGoal goal;
        goal.turtle_target_x = 0.0;
        goal.turtle_target_y = 0.0;
        goal.turtle_target_theta = g.response.yaw + 0.57;	// 底盘运动1.35弧度

        ac_->sendGoal(goal,  boost::bind(&MoveReq::doneCb, this, _1, _2), boost::bind(&MoveReq::activeCb, this), boost::bind(&MoveReq::feedbackCb,this,_1));
        sleep(1);

    }

    MoveReq::~MoveReq() { delete ac_;}


    void MoveReq::doneCb(const actionlib::SimpleClientGoalState& state,
                const learn_action::TurtleMoveResultConstPtr& result)
    {
        ROS_INFO("Yay! The TurtleMove is finished!");
        ros::shutdown();
    }

    void MoveReq::activeCb()
    {
        ROS_INFO("Goal just went active");
    }

    void MoveReq::feedbackCb(const learn_action::TurtleMoveFeedbackConstPtr& feedback)
    {
        ROS_INFO(" present_pose : %f  %f  %f", feedback->present_turtle_x,feedback->present_turtle_y,feedback->present_turtle_theta);
    }

}

 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "TurtleMove_client");

    RobotArm::MoveReq m;

    ros::spin();
    return 0;
}
