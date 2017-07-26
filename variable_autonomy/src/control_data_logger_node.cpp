/*!
 BLABLABLA
 */


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <iostream>

#include <geometry_msgs/Twist.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include <actionlib_msgs/GoalID.h>


class ControlDataLogger
{
public:
    ControlDataLogger() ;

private:

    void robotCmdVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void robotCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void computeCostCallback(const ros::TimerEvent&);
    void loaCallback(const std_msgs::Int8::ConstPtr& msg); // LAO topic


    double linear_error_, angular_error_ ;
    int previous_loa_;

    std_msgs::Float64 linear_error_msg_, angular_error_msg_, vector_vel_error_msg_;

    ros::NodeHandle n_;
    ros::Subscriber cmdvel_robot_sub_ , cmdvel_robot_optimal_sub_, control_mode_sub_;
    ros::Publisher linear_error_pub_, angluar_error_pub_, loa_changed_pub_, vector_vel_error_pub_;
    ros::Timer compute_cost_;

    geometry_msgs::Twist cmdvel_robot_, cmdvel_optimal_;
    std_msgs::Bool loa_changed_msg_;

};

ControlDataLogger::ControlDataLogger()
{

    previous_loa_ = 0;

    cmdvel_robot_sub_ = n_.subscribe("/cmd_vel", 5 , &ControlDataLogger::robotCmdVelCallback, this); // current velocity of the robot.
    cmdvel_robot_optimal_sub_ = n_.subscribe("/cmd_vel_optimal", 5 , &ControlDataLogger::robotCmdVelOptimalCallback, this); // The optimal velocity e.g. perfect move_base
    control_mode_sub_ = n_.subscribe("/control_mode", 5, &ControlDataLogger::loaCallback, this); // the LOA (from joystick)

    linear_error_pub_ = n_.advertise<std_msgs::Float64>("/linear_vel_error", 1);
    angluar_error_pub_ = n_.advertise<std_msgs::Float64>("/angular_error", 1);
    loa_changed_pub_ = n_.advertise<std_msgs::Bool>("/loa_has_changed", 1);
    vector_vel_error_pub_ = n_.advertise<std_msgs::Float64>("/vector_vel_error", 1);

    // The ros Duration controls the period in sec. that the cost, error etc will be computed. currently 10hz
    compute_cost_ = n_.createTimer(ros::Duration(0.2), &ControlDataLogger::computeCostCallback, this);
}


// logging currect cmdvel of robot
void ControlDataLogger::robotCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_robot_ = *msg;
}

// logging expert/optimal cmdvel
void ControlDataLogger::robotCmdVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_optimal_ = *msg;
}

void ControlDataLogger::loaCallback(const std_msgs::Int8::ConstPtr& msg)
{
    if (msg->data != previous_loa_)
    {
        previous_loa_ = msg->data;
        loa_changed_msg_.data = true;
        loa_changed_pub_.publish(loa_changed_msg_);
    }
}

// it computes the cmd vel error
void ControlDataLogger::computeCostCallback(const ros::TimerEvent&)
{

    double linear_error_component, angular_error_component;
    //linear error
    linear_error_ = cmdvel_optimal_.linear.x - cmdvel_robot_.linear.x;
    linear_error_msg_.data = fabs(linear_error_);
    if (linear_error_msg_.data > 0.1)
       {linear_error_msg_.data = 0.1; }
    linear_error_pub_.publish(linear_error_msg_);

    // angular error
    angular_error_ = fabs(cmdvel_optimal_.angular.z) - fabs(cmdvel_robot_.angular.z) ;
    angular_error_ = fabs(angular_error_);
    angular_error_msg_.data = angular_error_;
    angluar_error_pub_.publish(angular_error_msg_);

    // vector magnitute error
    linear_error_component = fabs(cmdvel_optimal_.linear.x - cmdvel_robot_.linear.x);
    if (linear_error_component > 0.1)
       {linear_error_component = 0.1; }
    angular_error_component = fabs(fabs(cmdvel_optimal_.angular.z) - fabs(cmdvel_robot_.angular.z));
    vector_vel_error_msg_.data = sqrt( pow(linear_error_component, 2) + pow(angular_error_component, 2) );
    vector_vel_error_pub_.publish(vector_vel_error_msg_);


}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mixed_initiative_controller");
    ControlDataLogger controller_obj;

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}

