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

//enum StringValue{Stop,
//           Teleoperation,
//           Autonomy};

//static std::map<std::string, StringValue> s_mapStringValues;

class ControlMixer
{
public:
    ControlMixer() ;

private:

    //void loaCallback(const std_msgs::Int8::ConstPtr& msg); // LAO topic
    void loaCallback(const std_msgs::String::ConstPtr& msg); // LAO topic
    void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg); // velocity from joystick
    void navCallback(const geometry_msgs::Twist::ConstPtr& msg); //velocity from the navigation e.g. move_base
    void miCommandCallback(const std_msgs::Bool::ConstPtr& msg);

    //int loa_;
    std::string loa_;
    bool valid_loa_;

    ros::NodeHandle n_;
    ros::Subscriber loa_sub_, vel_teleop_sub_, vel_nav_sub_ , mi_controller_sub_;
    ros::Publisher vel_for_robot_pub_ , cancelGoal_pub_ , loa_pub_, sound_pub_;

    geometry_msgs::Twist cmd_vel_for_robot_;
    actionlib_msgs::GoalID cancelGoal_;
    std_msgs::String loa_msg_;

};

ControlMixer::ControlMixer()
{
    valid_loa_ = true;
    loa_ = "Stop" ; //  stop/idle mode.

    cmd_vel_for_robot_.linear.x = 0;
    cmd_vel_for_robot_.angular.z = 0;


    vel_for_robot_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    cancelGoal_pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    loa_pub_ = n_.advertise<std_msgs::String>("/loa",1);

    loa_sub_ = n_.subscribe("/loa", 5, &ControlMixer::loaCallback, this); // the LOA (from joystick)
    vel_teleop_sub_ = n_.subscribe("/teleop/cmd_vel", 5, &ControlMixer::teleopCallback, this); // velocity coming from the teleoperation (Joystick)
    vel_nav_sub_ = n_.subscribe("/navigation/cmd_vel",5, &ControlMixer::navCallback, this); // velocity from the navigation e.g. move_base
    mi_controller_sub_ = n_.subscribe("/loa_change", 5, &ControlMixer::miCommandCallback, this); // MI controller LOA change command
}

// reads control mode topic to inform class internal variable
void ControlMixer::loaCallback(const std_msgs::String::ConstPtr& msg)
{
    do{
        valid_loa_ = false;
        ROS_INFO("Please choose a valid control mode.");


    if (msg->data == "Stop")
    {
        loa_ = "Stop";
        valid_loa_ = true;
        ROS_INFO("Stop robot");
        break;
    }
    else if (msg->data == "Teleoperation")
    {
        loa_ = "Teleoperation";
        valid_loa_ = true;
        cmd_vel_for_robot_.linear.x = 0;
        cmd_vel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_); // solves bug in which last auto msg if propagated in teleop
        ROS_INFO("Control mode: Teleoperation");
        break;
    }
    else if (msg->data == "Autonomy")
      //case 2
    {
        loa_ = "Autonomy";
        valid_loa_ = true;
        cmd_vel_for_robot_.linear.x = 0;
        cmd_vel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_); // solves bug in which last teleop msg if propagated in auto
        ROS_INFO("Control mode: Autonomy");
        break;
    }
    }while(0);
}


// Based on LOA choosen it allows for nav to have control of robot or not.
void ControlMixer::navCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

    if (loa_ == "Autonomy")
    {
        cmd_vel_for_robot_.linear.x = msg->linear.x;
        cmd_vel_for_robot_.angular.z = msg->angular.z;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_);
    }

    if (loa_ == "Stop")
    {
        cmd_vel_for_robot_.linear.x = 0;
        cmd_vel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_);
        cancelGoal_pub_.publish(cancelGoal_);

    }
}

// Based on LOA choosen it allows for pure teleop (operator) to have control of robot or not.
void ControlMixer::teleopCallback(const geometry_msgs::Twist::ConstPtr &msg)
{

    if (loa_ == "Teleoperation")
    {
        cmd_vel_for_robot_.linear.x = msg->linear.x;
        cmd_vel_for_robot_.angular.z = msg->angular.z;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_);
    }

    else if (loa_ == "Stop")
    {
        cmd_vel_for_robot_.linear.x = 0;
        cmd_vel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_);
        cancelGoal_pub_.publish(cancelGoal_);
    }
}

// reads the loa change command from MI controller and switchies to appropriate mode
void ControlMixer::miCommandCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == true && loa_ == "Teleoperation")
    {
        loa_msg_.data = "Autonomy";
        loa_pub_.publish(loa_msg_);
        cmd_vel_for_robot_.linear.x = 0;
        cmd_vel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_); // solves bug in which last auto msg if propagated in teleop

    }
    else if (msg->data == true && loa_ == "Autonomy")
    {
        loa_msg_.data = "Teleoperation";
        loa_pub_.publish(loa_msg_);
        cmd_vel_for_robot_.linear.x = 0;
        cmd_vel_for_robot_.angular.z = 0;
        vel_for_robot_pub_.publish(cmd_vel_for_robot_); // solves bug in which last auto msg if propagated in teleop
    }
    else if (msg->data == true && loa_ == "Stop")
    {
        loa_msg_.data = "Teleoperation";
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
