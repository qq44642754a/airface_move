#include "move_test.h"

MoveTest::MoveTest() : get_pose(0), move_finish(0), robot_move(0), getflagsuccess(0), move_base("move_base", true)

{
  ros::AsyncSpinner spinner(1);
  spinner.start();
  pose_sub = nh_.subscribe("/visualization_marker", 10, &MoveTest::poseCallback, this);
  pose = nh_.serviceClient<airface_drive_msgs::getpose>("/airface_driver/get_tf_pose");
}

MoveTest::~MoveTest()
{
  cout << "delete the class" << endl;
}

void MoveTest::poseCallback(const visualization_msgs::Marker &marker_tmp)
{
  if (robot_move == 0)
  {
    ROS_INFO("reading the pose of Marker");
    Marker_pose_tmp.header = marker_tmp.header;
    cout << Marker_pose_tmp.header << endl;
    Marker_pose_tmp.pose = marker_tmp.pose;
    Marker_pose_tmp.pose.position.z -= 0.8; //0.8m in front of the Marker
    if (Marker_pose_tmp.pose.position.x != 0)
    {
      get_pose = 1;
    }
  }
}

void MoveTest::transform_tf()
{

  if (get_pose == 1)
    ROS_INFO("get the pose of Marker, tansform now"); // Marker pose in camera link to pose in map
  {
    try
    {
      listener.transformPose("/map", Marker_pose_tmp, Marker_pose);
      getflagsuccess = 1;
      robot_move = 1;
      ROS_INFO("Transform successed");
    }
    catch (tf::TransformException &ex)
    {
      ros::Duration(0.5).sleep();
      getflagsuccess = 0;
      std::cout << "Transform failed, other try" << std::endl;
    }
    if (getflagsuccess)
    {
      airface_drive_msgs::getpose g;
      pose.call(g);
      Marker_pose.pose.orientation = tf::createQuaternionMsgFromYaw(g.response.yaw);
      cout << "Target pose was :\n"
           << Marker_pose.pose.position << endl;
      Marker_pose.pose.position.z = 0;
    }
  }
}

void MoveTest::goSP()
{

  //connet to the Server, 5s limit
  while (!move_base.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for move_base action server...");
  }

  ROS_INFO("Connected to move base server");

  //set the targetpose
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = Marker_pose.pose.position.x;
  goal.target_pose.pose.position.y = Marker_pose.pose.position.y;

  goal.target_pose.pose.orientation.z = Marker_pose.pose.orientation.z;
  goal.target_pose.pose.orientation.w = Marker_pose.pose.orientation.w;

  ROS_INFO("Sending goal");
  move_base.sendGoal(goal);

  move_base.waitForResult();

  if (move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal succeeded!");
  else
    ROS_INFO("Goal failed");
}

void MoveTest::initMove()
{

  ros::AsyncSpinner spinner(1);
  spinner.start();
  transform_tf();
  if (getflagsuccess == 1)
  {
    ROS_INFO("get the Targetpose. now go to the Targetpose");
    goSP();
    sleep(2);
    move_finish = 1;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_test");
  MoveTest movetest;

  while (ros::ok())
  {
    ros::spinOnce();
    movetest.initMove();
    if (movetest.move_finish == 1)
    {
      ros::shutdown();
    }
  }
  return 0;
}
