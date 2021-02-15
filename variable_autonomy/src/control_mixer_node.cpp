/*!
 BLABLABLA
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <iostream>
#include <algorithm>

#include <geometry_msgs/Twist.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include <actionlib_msgs/GoalID.h>

class ControlMixer
{
public:
    ControlMixer();

private:
    void loaCallback(const std_msgs::Int8::ConstPtr &msg);          // negotiated LOA callback topic or final LOA topic coming from an arbitration agent
    void teleopCallback(const geometry_msgs::Twist::ConstPtr &msg); // command velocity from joystick/teleop
    void navCallback(const geometry_msgs::Twist::ConstPtr &msg);    //velocity from the navigation e.g. move_base

    int loa_;
    bool valid_loa_;

    ros::NodeHandle n_;
    ros::Subscriber loa_sub_, vel_teleop_sub_, vel_nav_sub_;
    ros::Publisher vel_for_robot_pub_, cancelGoal_pub_, loa_pub_, sound_pub_;

    geometry_msgs::Twist cmd_vel_for_robot_;
    actionlib_msgs::GoalID cancelGoal_;
    std_msgs::Int8 loa_msg_;
};

ControlMixer::ControlMixer()
{
    valid_loa_ = true;
    loa_ = 1; // start in teleop.

    cmd_vel_for_robot_.linear.x = 0;
    cmd_vel_for_robot_.angular.z = 0;

    vel_for_robot_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // this send to the robot either the teleop velocity or the AI's nav planner velocity, depending on LOA
    cancelGoal_pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    loa_pub_ = n_.advertise<std_msgs::Int8>("/loa_new", 1);

    loa_sub_ = n_.subscribe("/nemi/negotiated_loa", 5, &ControlMixer::loaCallback, this);      // the negotiated LOA topic or final LOA topic coming from an arbitration agent
    vel_teleop_sub_ = n_.subscribe("/teleop/cmd_vel", 5, &ControlMixer::teleopCallback, this); // velocity coming from the teleoperation (Joystick)
    vel_nav_sub_ = n_.subscribe("/navigation/cmd_vel", 5, &ControlMixer::navCallback, this);   // velocity from the AI navigation e.g. move_base
}

// subscribes/reads the negotiated LOA and updates class internal variable while publishing it under "/loa"
void ControlMixer::loaCallback(const std_msgs::Int8::ConstPtr &msg)
{
    switch (msg->data)
    {
    case 0:
    {
        loa_ = 0;
        valid_loa_ = true;
         loa_msg_.data = loa_;
        loa_pub_.publish(loa_msg_);
        ROS_INFO("Stop robot");
        break;
    }
    case 1:
    {
        loa_ = 1;
        valid_loa_ = true;
        cmd_vel_for_robot_.linear.x = 0;
        cmd_vel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_); // solves bug in which last auto msg if propagated in teleop
        loa_msg_.data = loa_;
        loa_pub_.publish(loa_msg_);
        ROS_INFO("Control mode: Teleoperation");
        break;
    }
    case 2:
    {
        loa_ = 2;
        valid_loa_ = true;
        cmd_vel_for_robot_.linear.x = 0;
        cmd_vel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_); // solves bug in which last teleop msg if propagated in auto
        loa_msg_.data = loa_;
        loa_pub_.publish(loa_msg_);
        ROS_INFO("Control mode: Autonomy");
        break;
    }
    default:
    {
        valid_loa_ = false;
        ROS_INFO("Please choose a valid control mode.");
    }
    }
}

// Based on LOA choosen it allows for AI's NAV planner commands to be sent to robot
void ControlMixer::navCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if (loa_ == 2)
    {
        cmd_vel_for_robot_.linear.x = msg->linear.x;
        cmd_vel_for_robot_.angular.z = msg->angular.z;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_);
    }

    if (loa_ == 0)
    {
        cmd_vel_for_robot_.linear.x = 0;
        cmd_vel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_);
        cancelGoal_pub_.publish(cancelGoal_);
    }
}

// Based on LOA choosen it allows for pure teleop (operator) velocity commands to to be sent to robot.
void ControlMixer::teleopCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if (loa_ == 1)
    {
        cmd_vel_for_robot_.linear.x = msg->linear.x;
        cmd_vel_for_robot_.angular.z = msg->angular.z;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_);
    }

    else if (loa_ == 0)
    {
        cmd_vel_for_robot_.linear.x = 0;
        cmd_vel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_);
        cancelGoal_pub_.publish(cancelGoal_);
    }
}

// // reads the loa change command from MI controller and switchies to appropriate mode
// void ControlMixer::miCommandCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//     if (msg->data == true && loa_ == 1)
//     {
//         loa_msg_.data = 2;
//         loa_pub_.publish(loa_msg_);
//         cmd_vel_for_robot_.linear.x = 0;
//         cmd_vel_for_robot_.angular.z = 0;
//         vel_for_robot_pub_.publish(cmd_vel_for_robot_); // solves bug in which last auto msg if propagated in teleop

//     }
//     else if (msg->data == true && loa_ == 2)
//     {
//         loa_msg_.data = 1;
//         loa_pub_.publish(loa_msg_);
//         cmd_vel_for_robot_.linear.x = 0;
//         cmd_vel_for_robot_.angular.z = 0;
//         vel_for_robot_pub_.publish(cmd_vel_for_robot_); // solves bug in which last auto msg if propagated in teleop
//     }
//     else if (msg->data == true && loa_ == 0)
//     {
//         loa_msg_.data = 1;
//         loa_pub_.publish(loa_msg_);
//     }
// }

// Main function stuff
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_mixer_node");
    ControlMixer mixer_obj;

    ros::Rate r(20); // 20 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
