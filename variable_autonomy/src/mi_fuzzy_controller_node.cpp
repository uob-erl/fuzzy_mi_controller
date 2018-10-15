
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
#include "std_msgs/Float64.h"
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>

#include <fl/Headers.h>

class ControlDataLogger
{
public:
    ControlDataLogger(fl::Engine *engine) ;

private:

    void loaCallback(const std_msgs::Int8::ConstPtr& msg);
    void robotVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void computeCostCallback(const ros::TimerEvent&);
    void goalResultCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& goalResult);

    int loa_, number_timesteps_error_ , count_timesteps_error_, number_timesteps_vel_, count_timesteps_vel_, previous_loa_  ;
    bool valid_loa_, mi_active_;
    double error_sum_, error_average_, velocity_sum_, vel_error_average_,  a_,vel_error_ , vel_error_threshold_, decision_;
    std_msgs::Bool loa_change_, loa_changed_msg_;
    std_msgs::Float64 error_average_msg_, vel_average_msg_;
    std_msgs::Int8 ai_switch_count_msg_;

    ros::NodeHandle n_;
    ros::Subscriber loa_sub_ ,vel_robot_sub_ , vel_robot_optimal_sub_, sub_goal_status_;
    ros::Publisher loa_pub_, loa_change_pub_, vel_error_pub_, vel_error_average_pub_, ai_switch_count_pub_, loa_changed_pub_;
    ros::Timer compute_cost_;

    geometry_msgs::Twist cmdvel_robot_, cmdvel_for_robot_, cmdvel_optimal_;
    actionlib_msgs::GoalID cancelGoal_;

    fl::Engine* engine_;

};

ControlDataLogger::ControlDataLogger(fl::Engine* engine)
{

    loa_change_.data = false;
    valid_loa_ = false;
    a_ = 0.06; // smoothing factor [0,1]
    vel_error_threshold_ = 0;

    number_timesteps_error_ = 16; // # of time steps used to initialize average
    count_timesteps_error_ = 1; // counts the # of time steps used to initialize average
    number_timesteps_vel_ = 16;
    count_timesteps_vel_ = 1;
    ai_switch_count_msg_.data = 0;
    previous_loa_ = 0;

    loa_change_pub_ = n_.advertise<std_msgs::Bool>("/loa_change", 1);
    vel_error_pub_ = n_.advertise<std_msgs::Float64>("/vel_error", 1);
    vel_error_average_pub_ = n_.advertise<std_msgs::Float64>("/vel_error_average", 1);
    ai_switch_count_pub_ = n_.advertise<std_msgs::Int8>("/ai_switch_count", 1);
    loa_changed_pub_ = n_.advertise<std_msgs::Bool>("/loa_has_changed", 1);
    sub_goal_status_ = n_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base_optimal/status", 1, &ControlDataLogger::goalResultCallBack,this);

    loa_sub_ = n_.subscribe("/control_mode", 5, &ControlDataLogger::loaCallback, this); // the current LOA
    vel_robot_sub_ = n_.subscribe("/cmd_vel", 5 , &ControlDataLogger::robotVelCallback, this); // current velocity of the robot.
    vel_robot_optimal_sub_ = n_.subscribe("/cmd_vel_optimal", 5 , &ControlDataLogger::robotVelOptimalCallback, this); // The optimal velocity e.g. perfect move_base

    // The ros Duration controls the period in sec. that the cost will compute. currently 10hz
    compute_cost_ = n_.createTimer(ros::Duration(0.2), &ControlDataLogger::computeCostCallback, this);

    engine_ = engine;
    FL_LOG("Fuzzy engine created");

}

// takes the goal result/status from move_base_optimal
void ControlDataLogger::goalResultCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{

    if (!msg->status_list.empty())
    {
        actionlib_msgs::GoalStatus goalStatus;
        goalStatus = msg->status_list[0];


        if (goalStatus.status == 1)
        {
            ROS_INFO("active goal in progress");
            mi_active_ = 1;
        }

        else if (goalStatus.status == 3)
        {
            ROS_INFO("robot moved succefuly to goal");
            mi_active_ = 0;
        }

        else if (goalStatus.status == 2 )
        {
            ROS_INFO("Goal was cancelled");
            mi_active_ = 0;
        }

        else if (goalStatus.status == 4 )
        {
            ROS_INFO("Goal was aborded");
            mi_active_ = 0;

        }

        else {
            ROS_INFO("What happened? Status Something else?? Check /move_base_optimal/status for code denoting status");
            mi_active_ = 0;
        }
    }
}

void ControlDataLogger::loaCallback(const std_msgs::Int8::ConstPtr& msg)
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
        ROS_INFO("Control mode: Teleoperation");
        break;
    }
    case 2:
    {
        loa_ = 2;
        valid_loa_ = true;
        ROS_INFO("Control mode: Autonomy");
        break;
    }
    default:
    {
        valid_loa_ = false;
        ROS_INFO("Please choose a valid control mode.");
    }
    }

    if (msg->data != previous_loa_)
    {
        previous_loa_ = msg->data;
        loa_changed_msg_.data = true;
        loa_changed_pub_.publish(loa_changed_msg_);
    }

}




void ControlDataLogger::robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_robot_ = *msg;

}

void ControlDataLogger::robotVelOptimalCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmdvel_optimal_ = *msg;
}

// Where magic happens, it computes the cost, judging to switch LAO
void ControlDataLogger::computeCostCallback(const ros::TimerEvent&)
{

    if (mi_active_ = 1)
    {
        vel_error_ = cmdvel_optimal_.linear.x - cmdvel_robot_.linear.x;
        vel_error_ = fabs(vel_error_);

        if (vel_error_ > 0.1)   //bounds error
        {vel_error_ = 0.1; }

        // calculates the moving average initialization for velocity
        if (count_timesteps_vel_ <= number_timesteps_vel_)
        {
            velocity_sum_ += cmdvel_robot_.linear.x;
            vel_error_average_ = velocity_sum_ / number_timesteps_vel_;
            count_timesteps_vel_++;
        }

        // calculates exponential moving average for current velocity
        else if (count_timesteps_vel_ > number_timesteps_vel_)
        {
            vel_error_average_ = a_ * cmdvel_robot_.linear.x + (1-a_) * vel_error_average_;
            engine_->setInputValue("speed", vel_error_average_);
        }

        // calculates the average error used to initialize exponential moving average
        if (count_timesteps_error_ <= number_timesteps_error_)
        {
            error_sum_ += vel_error_;
            error_average_ = error_sum_ / number_timesteps_error_;
            count_timesteps_error_++;
        }


        // calculates  exponential moving average for error
        else if (count_timesteps_error_ > number_timesteps_error_)
        {
            error_average_ = a_ * vel_error_ + (1-a_) * error_average_;
            engine_->setInputValue("error", error_average_);
            engine_->setInputValue("speed", cmdvel_robot_.linear.x);
            engine_->process();
            decision_ = engine_->getOutputValue("change_LOA");
            FL_LOG("error = " << fl::Op::str(error_average_) );
            FL_LOG("speed = " << fl::Op::str(cmdvel_robot_.linear.x) ) ;
            FL_LOG("Decision = " << fl::Op::str(engine_->getOutputValue("change_LOA") ) );


            if ( (decision_ > vel_error_threshold_) && (loa_change_.data == false) )
            {
                loa_change_.data = true;
                loa_change_pub_.publish(loa_change_);

                ai_switch_count_msg_.data++ ;
                ai_switch_count_pub_.publish(ai_switch_count_msg_);

                count_timesteps_error_ = 1; // enables re-initializaion of moving average by reseting count
                loa_change_.data = false; // resets loa_change flag
                error_sum_ = 0; // resets sumation of errors for initial estimate
                ros::Duration(10).sleep();
            }
            else if ((decision_ < vel_error_threshold_) && loa_change_.data == true)
            {
                loa_change_.data = false;
                loa_change_pub_.publish(loa_change_);
            }

        }

        error_average_msg_.data = error_average_;
        vel_error_pub_.publish(error_average_msg_);

        vel_average_msg_.data = vel_error_average_;
        vel_error_average_pub_.publish(vel_average_msg_);
    }

}


int main(int argc, char *argv[])
{

    // Fuzzylite stuff

    fl::Engine* engine = new fl::Engine;
    engine->setName("controller");

    fl::InputVariable* inputVariable1 = new fl::InputVariable;
    inputVariable1->setEnabled(true);
    inputVariable1->setName("error");
    inputVariable1->setRange(0.000, 0.100);
    inputVariable1->addTerm(new fl::Trapezoid("small", 0.000, 0.000, 0.035, 0.060));
    inputVariable1->addTerm(new fl::Trapezoid("medium", 0.045, 0.055, 0.065, 0.080));
    inputVariable1->addTerm(new fl::Trapezoid("large", 0.065, 0.085, 0.100, 0.100));
    engine->addInputVariable(inputVariable1);

    fl::InputVariable* inputVariable2 = new fl::InputVariable;
    inputVariable2->setEnabled(true);
    inputVariable2->setName("speed");
    inputVariable2->setRange(-0.400, 0.400);
    inputVariable2->addTerm(new fl::Trapezoid("reverse", -0.400, -0.400, -0.030, -0.020));
    inputVariable2->addTerm(new fl::Triangle("zero", -0.030, 0.000, 0.030));
    inputVariable2->addTerm(new fl::Trapezoid("forward", 0.020, 0.030, 0.400, 0.400));
    engine->addInputVariable(inputVariable2);

    fl::OutputVariable* outputVariable = new fl::OutputVariable;
    outputVariable->setEnabled(true);
    outputVariable->setName("change_LOA");
    outputVariable->setRange(-1.000, 1.000);
    outputVariable->fuzzyOutput()->setAggregation(new fl::Maximum); // the old setAccumulation is the new Aggregation?
    outputVariable->setDefuzzifier(new fl::LargestOfMaximum(200));
    outputVariable->setDefaultValue(fl::nan);

    outputVariable->addTerm(new fl::Triangle("change", 0.000, 1.000, 1.000));
    outputVariable->addTerm(new fl::Triangle("no_change", -1.000, -1.000, 0.000));
    engine->addOutputVariable(outputVariable);

    fl::RuleBlock* ruleBlock = new fl::RuleBlock;
    ruleBlock->setEnabled(true);
    ruleBlock->setName("");
    ruleBlock->setConjunction(new fl::Minimum);
    ruleBlock->setDisjunction(new fl::Maximum);
    ruleBlock->setImplication(new fl::Minimum);
    // ruleBlock->setActivation(new fl::Threshold);This is now called implication operator
    ruleBlock->addRule(fl::Rule::parse("if error is small or error is medium then change_LOA is no_change", engine));
    ruleBlock->addRule(fl::Rule::parse("if error is large and speed is not reverse then change_LOA is change", engine));
    ruleBlock->addRule(fl::Rule::parse("if speed is reverse and error is large then change_LOA is no_change", engine));
    engine->addRuleBlock(ruleBlock);





    //-------------------------------------------------------------------///


    ros::init(argc, argv, "mixed_initiative_controller");
    ControlDataLogger controller_obj(engine);

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}

