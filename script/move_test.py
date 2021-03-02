#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  

# 节点初始化 
rospy.init_node('move_test', anonymous=True)  
  
# 订阅move_base服务器的消息  
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

rospy.loginfo("Waiting for move_base action server...")  

# 等待连接服务器，5s等待时间限制 
while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
    rospy.loginfo("Connected to move base server")  

# 设定目标点  
#target = Pose(Point(-1.7014, 2.68, 0), Quaternion(0.000, 0.000, -0.9999, 0.0157))  
target = Pose(Point( -1.11014134933, 5.23633609861, 0), Quaternion(0.000, 0.000,  -0.9961, 0.0877)) 
#target = Pose(Point(-1.5848317, 5.26115197, 0), Quaternion(0.000, 0.000, -0.9999, 0.0157)) 
goal = MoveBaseGoal()  
goal.target_pose.pose = target  
goal.target_pose.header.frame_id = 'map'  
goal.target_pose.header.stamp = rospy.Time.now()  

rospy.loginfo("Going to: " + str(target))  

# 向目标进发  
move_base.send_goal(goal)  

# 五分钟时间限制  
finished_within_time = move_base.wait_for_result(rospy.Duration(300))   

# 查看是否成功到达  
if not finished_within_time:  
    move_base.cancel_goal()  
    rospy.loginfo("Timed out achieving goal")  
else:  
    state = move_base.get_state()  
    if state == GoalStatus.SUCCEEDED:  
        rospy.loginfo("Goal succeeded!")
    else:  
      rospy.loginfo("Goal failed！ ")  

