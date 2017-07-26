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
    ControlMixer() ;

private:

    void loaCallback(const std_msgs::Int8::ConstPtr& msg); // LAO topic
    void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg); // velocity from joystick
    void navCallback(const geometry_msgs::Twist::ConstPtr& msg); //velocity from the navigation e.g. move_base
    void miCommandCallback(const std_msgs::Bool::ConstPtr& msg);

    int loa_;
    bool valid_loa_;

    ros::NodeHandle n_;
    ros::Subscriber control_mode_sub_, vel_teleop_sub_, vel_nav_sub_ , mi_controller_sub_;
    ros::Publisher vel_for_robot_pub_ , cancelGoal_pub_ , loa_pub_, sound_pub_;

    geometry_msgs::Twist cmdvel_for_robot_;
    actionlib_msgs::GoalID cancelGoal_;
    std_msgs::Int8 loa_msg_;

};

ControlMixer::ControlMixer()
{
    valid_loa_ = true;
    loa_ = 0 ; //  stop/idle mode.

    cmdvel_for_robot_.linear.x = 0;
    cmdvel_for_robot_.angular.z = 0;


    vel_for_robot_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    cancelGoal_pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    loa_pub_ = n_.advertise<std_msgs::Int8>("/control_mode",1);

    control_mode_sub_ = n_.subscribe("/control_mode", 5, &ControlMixer::loaCallback, this); // the LOA (from joystick)
    vel_teleop_sub_ = n_.subscribe("/teleop/cmd_vel", 5, &ControlMixer::teleopCallback, this); // velocity coming from the teleoperation (Joystick)
    vel_nav_sub_ = n_.subscribe("/navigation/cmd_vel",5, &ControlMixer::navCallback, this); // velocity from the navigation e.g. move_base
    mi_controller_sub_ = n_.subscribe("/loa_change", 5, &ControlMixer::miCommandCallback, this); // MI controller LOA change command
}

// reads control mode topic to inform class internal variable
void ControlMixer::loaCallback(const std_msgs::Int8::ConstPtr& msg)
{

    switch (msg->data)
    {
    case 0:
    {
        loa_ = 0;
        valid_loa_ = true;
        ROS_INFO("Stop robot");
        break;
    }
    case 1:
    {
        loa_ = 1;
        valid_loa_ = true;
        cmdvel_for_robot_.linear.x = 0;
        cmdvel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmdvel_for_robot_); // solves bug in which last auto msg if propagated in teleop
        ROS_INFO("Control mode: Teleoperation");
        break;
    }
    case 2:
    {
        loa_ = 2;
        valid_loa_ = true;
        cmdvel_for_robot_.linear.x = 0;
        cmdvel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmdvel_for_robot_); // solves bug in which last teleop msg if propagated in auto
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

// Based on LOA choosen it allows for nav to have control of robot or not.
void ControlMixer::navCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

    if (loa_ == 2)
    {
        cmdvel_for_robot_.linear.x = msg->linear.x;
        cmdvel_for_robot_.angular.z = msg->angular.z;
        vel_for_robot_pub_.publish(cmdvel_for_robot_);
    }

    if (loa_ == 0)
    {
        cmdvel_for_robot_.linear.x = 0;
        cmdvel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmdvel_for_robot_);
        cancelGoal_pub_.publish(cancelGoal_);

    }
}

// Based on LOA choosen it allows for pure teleop (operator) to have control of robot or not.
void ControlMixer::teleopCallback(const geometry_msgs::Twist::ConstPtr &msg)
{

    if (loa_ == 1)
    {
        cmdvel_for_robot_.linear.x = msg->linear.x;
        cmdvel_for_robot_.angular.z = msg->angular.z;
        vel_for_robot_pub_.publish(cmdvel_for_robot_);
    }

    else if (loa_ == 0)
    {
        cmdvel_for_robot_.linear.x = 0;
        cmdvel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmdvel_for_robot_);
        cancelGoal_pub_.publish(cancelGoal_);
    }
}

// reads the loa change command from MI controller and switchies to appropriate mode
void ControlMixer::miCommandCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == true && loa_ == 1)
    {
        loa_msg_.data = 2;
        loa_pub_.publish(loa_msg_);
        cmdvel_for_robot_.linear.x = 0;
        cmdvel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmdvel_for_robot_); // solves bug in which last auto msg if propagated in teleop

    }
    else if (msg->data == true && loa_ == 2)
    {
        loa_msg_.data = 1;
        loa_pub_.publish(loa_msg_);
        cmdvel_for_robot_.linear.x = 0;
        cmdvel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmdvel_for_robot_); // solves bug in which last auto msg if propagated in teleop
    }
    else if (msg->data == true && loa_ == 0)
    {
        loa_msg_.data = 1;
        loa_pub_.publish(loa_msg_);
    }
}



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

