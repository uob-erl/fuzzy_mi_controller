
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
#include "std_msgs/Float32.h"
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>

#include <fl/Headers.h>

class MixedInitiativeControllerDeepgaze
{
public:
MixedInitiativeControllerDeepgaze(fl::Engine *engine);

private:

void loaCallback(const std_msgs::Int8::ConstPtr& msg);
void robotVelExpertCallback(const geometry_msgs::Twist::ConstPtr& msg);
void robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
void computeCostCallback(const ros::TimerEvent&);
void goalResultCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& goalResult);
void deepgazeCallback(const std_msgs::Float32::ConstPtr& msg);

int loa_, number_timesteps_error_, count_timesteps_error_, number_timesteps_vel_, count_timesteps_vel_, previous_loa_;
bool valid_loa_, mi_active_, deepgaze_active_;
double error_sum_, error_average_, velocity_sum_, vel_error_average_,  a_,vel_error_, vel_error_threshold_, decision_;
std_msgs::Bool loa_change_, loa_changed_msg_;
std_msgs::Float64 error_average_msg_, vel_average_msg_;
std_msgs::Int8 ai_switch_count_msg_;
std_msgs::Float32 cognitive_availability_;

ros::NodeHandle n_;
ros::Subscriber loa_sub_,vel_robot_sub_, vel_robot_expert_sub_, sub_goal_status_, deepgaze_sub_;
ros::Publisher loa_pub_, loa_change_pub_, goal_directed_motion_error_pub_, goal_directed_motion_error_average_pub_, ai_switch_count_pub_, loa_changed_pub_;
ros::Timer compute_cost_;

geometry_msgs::Twist cmd_vel_robot_, cmdvel_for_robot_, cmd_vel_expert_;
actionlib_msgs::GoalID cancelGoal_;

fl::Engine* engine_;

};

MixedInitiativeControllerDeepgaze::MixedInitiativeControllerDeepgaze(fl::Engine* engine)
{

        loa_change_.data = false;
        valid_loa_ = false;
        ai_switch_count_msg_.data = 0;
        previous_loa_ = 0;

        a_ = 0.06; // smoothing factor [0,1]
        vel_error_threshold_ = 0;
        number_timesteps_error_ = 16; // # of time steps used to initialize average
        count_timesteps_error_ = 1; // counts the # of time steps used to initialize average
        number_timesteps_vel_ = 16;
        count_timesteps_vel_ = 1;


        loa_change_pub_ = n_.advertise<std_msgs::Bool>("/loa_change", 1);
        goal_directed_motion_error_pub_ = n_.advertise<std_msgs::Float64>("/goal_directed_motion/error", 1);
        goal_directed_motion_error_average_pub_ = n_.advertise<std_msgs::Float64>("/goal_directed_motion/error_average", 1);
        ai_switch_count_pub_ = n_.advertise<std_msgs::Int8>("/ai_switch_count", 1);
        loa_changed_pub_ = n_.advertise<std_msgs::Bool>("/loa_has_changed", 1);
        sub_goal_status_ = n_.subscribe<actionlib_msgs::GoalStatusArray>("/expert_move_base/status", 1, &MixedInitiativeControllerDeepgaze::goalResultCallBack,this);

        loa_sub_ = n_.subscribe("/loa", 5, &MixedInitiativeControllerDeepgaze::loaCallback, this); // the current LOA
        vel_robot_sub_ = n_.subscribe("/cmd_vel", 5, &MixedInitiativeControllerDeepgaze::robotVelCallback, this); // current velocity of the robot.
        vel_robot_expert_sub_ = n_.subscribe("/cmd_vel_expert", 5, &MixedInitiativeControllerDeepgaze::robotVelExpertCallback, this); // The expert suggested velocity e.g. perfect move_base
        deepgaze_sub_ = n_.subscribe("/cognitive_availability", 5, &MixedInitiativeControllerDeepgaze::deepgazeCallback, this);

        // The ros Duration controls the period in sec. that the cost will compute. currently 10hz
        compute_cost_ = n_.createTimer(ros::Duration(0.2), &MixedInitiativeControllerDeepgaze::computeCostCallback, this);

        engine_ = engine;
        FL_LOG("Fuzzy engine created");

}

void MixedInitiativeControllerDeepgaze::deepgazeCallback(const std_msgs::Float32::ConstPtr &msg)
{
     cognitive_availability_ = *msg;
     deepgaze_active_ = 1;
//     ROS_INFO("cognitive availability: %d", cognitive_availability_.data);
//     ROS_INFO("deepgaze active: %d", deepgaze_active_);
}

void MixedInitiativeControllerDeepgaze::robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
        cmd_vel_robot_ = *msg;

}

void MixedInitiativeControllerDeepgaze::robotVelExpertCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
        cmd_vel_expert_ = *msg;
}

// takes the goal result/status from the expert move_base
void MixedInitiativeControllerDeepgaze::goalResultCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
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
                        ROS_INFO("What happened? Status Something else?? Check expert move base status topic for code denoting status");
                        mi_active_ = 0;
                }
        }
}

void MixedInitiativeControllerDeepgaze::loaCallback(const std_msgs::Int8::ConstPtr& msg)
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


// Where magic happens, it computes the cost, judging to switch LAO
void MixedInitiativeControllerDeepgaze::computeCostCallback(const ros::TimerEvent&)
{

        if (mi_active_ == 1 and deepgaze_active_ == 1)
        {
                ROS_INFO("bike");
                vel_error_ = cmd_vel_expert_.linear.x - cmd_vel_robot_.linear.x;
                vel_error_ = fabs(vel_error_);

                if (vel_error_ > 0.1) //bounds error
                {vel_error_ = 0.1; }

                // calculates the moving average initialization for velocity
                if (count_timesteps_vel_ <= number_timesteps_vel_)
                {
                        velocity_sum_ += cmd_vel_robot_.linear.x;
                        vel_error_average_ = velocity_sum_ / number_timesteps_vel_;
                        count_timesteps_vel_++;
                }

                // calculates exponential moving average for current velocity
                else if (count_timesteps_vel_ > number_timesteps_vel_)
                {
                        vel_error_average_ = a_ * cmd_vel_robot_.linear.x + (1-a_) * vel_error_average_;
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
                        // input values to fuzzy engine
                        engine_->setInputValue("error", error_average_);
                        engine_->setInputValue("speed", cmd_vel_robot_.linear.x);
                        engine_->setInputValue("cognitive_availability", cognitive_availability_.data);
                        engine_->setInputValue("current_loa",loa_);
                        engine_->process();
                        decision_ = engine_->getOutputValue("change_LOA");
                        FL_LOG("error = " << fl::Op::str(error_average_) );
//                        FL_LOG("speed = " << fl::Op::str(cmd_vel_robot_.linear.x) );
                        FL_LOG("cognitive_availability = " << fl::Op::str(cognitive_availability_.data) );
//                        FL_LOG("current_loa = " << fl::Op::str(loa_) );
                        FL_LOG("Decision = " << fl::Op::str(engine_->getOutputValue("change_LOA") ) );


                        if ( (decision_ > vel_error_threshold_) && (loa_change_.data == false) )
                        {
                                loa_change_.data = true;
                                loa_change_pub_.publish(loa_change_);

                                ai_switch_count_msg_.data++;
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
                goal_directed_motion_error_pub_.publish(error_average_msg_);

                vel_average_msg_.data = vel_error_average_;
                goal_directed_motion_error_average_pub_.publish(vel_average_msg_);
        }

}


int main(int argc, char *argv[])
{

        // Fuzzylite stuff

        fl::Engine* engine = new fl::Engine;
        engine->setName("controller");

        // define fuzzy sets and input variables

        fl::InputVariable* error = new fl::InputVariable;
        error->setName("error");
        error->setDescription("");
        error->setEnabled(true);
        error->setRange(0.000, 0.100);
        error->setLockValueInRange(false);
        error->addTerm(new fl::Trapezoid("small", 0.000, 0.000, 0.035, 0.060));
        error->addTerm(new fl::Trapezoid("medium", 0.045, 0.055, 0.065, 0.080));
        error->addTerm(new fl::Trapezoid("large", 0.065, 0.085, 0.100, 0.100));
        engine->addInputVariable(error);

        fl::InputVariable* speed = new fl::InputVariable;
        speed->setName("speed");
        speed->setDescription("");
        speed->setEnabled(true);
        speed->setRange(-0.400, 0.400);
        speed->setLockValueInRange(false);
        speed->addTerm(new fl::Trapezoid("reverse", -0.400, -0.400, -0.030, -0.020));
        speed->addTerm(new fl::Triangle("zero", -0.030, 0.000, 0.030));
        speed->addTerm(new fl::Trapezoid("forward", 0.020, 0.030, 0.400, 0.400));
        engine->addInputVariable(speed);

        fl::InputVariable* current_loa = new fl::InputVariable;
        current_loa->setName("current_loa");
        current_loa->setDescription("");
        current_loa->setEnabled(true);
        current_loa->setRange(0.000, 3.000);
        current_loa->setLockValueInRange(false);
        current_loa->addTerm(new fl::Rectangle("teleop", 0.900, 1.100));
        current_loa->addTerm(new fl::Rectangle("auto", 1.900, 2.100));
        engine->addInputVariable(current_loa);

        fl::InputVariable* cognitive_availability = new fl::InputVariable;
        cognitive_availability->setName("cognitive_availability");
        cognitive_availability->setDescription("");
        cognitive_availability->setEnabled(true);
        cognitive_availability->setRange(0.000, 50.000);
        cognitive_availability->setLockValueInRange(false);
        cognitive_availability->addTerm(new fl::Trapezoid("available", 0.000, 0.000, 18.000, 22.000));
        cognitive_availability->addTerm(new fl::Trapezoid("unavailable", 18.000, 22.000, 50.000, 50.000));
        engine->addInputVariable(cognitive_availability);

//        fl::InputVariable* inputVariable1 = new fl::InputVariable;
//        inputVariable1->setEnabled(true);
//        inputVariable1->setName("error");
//        inputVariable1->setRange(0.000, 0.100);
//        inputVariable1->addTerm(new fl::Trapezoid("small", 0.000, 0.000, 0.035, 0.060));
//        inputVariable1->addTerm(new fl::Trapezoid("medium", 0.045, 0.055, 0.065, 0.080));
//        inputVariable1->addTerm(new fl::Trapezoid("large", 0.065, 0.085, 0.100, 0.100));
//        engine->addInputVariable(inputVariable1);

//        fl::InputVariable* inputVariable2 = new fl::InputVariable;
//        inputVariable2->setEnabled(true);
//        inputVariable2->setName("speed");
//        inputVariable2->setRange(-0.400, 0.400);
//        inputVariable2->addTerm(new fl::Trapezoid("reverse", -0.400, -0.400, -0.030, -0.020));
//        inputVariable2->addTerm(new fl::Triangle("zero", -0.030, 0.000, 0.030));
//        inputVariable2->addTerm(new fl::Trapezoid("forward", 0.020, 0.030, 0.400, 0.400));
//        engine->addInputVariable(inputVariable2);

//        fl::InputVariable* inputVariable3 = new fl::InputVariable;
//        inputVariable3->setEnabled(true);
//        inputVariable3->setName("cognitive_availability");
//        inputVariable3->setRange(0.000, 5.000);
//        inputVariable3->addTerm(new fl::Trapezoid("available", 0.000,  0.000, 3.000, 3.000));
//        inputVariable3->addTerm(new fl::Trapezoid("not_available", 3.000, 4.000, 5.000, 5.000));
//        engine->addInputVariable(inputVariable3);

        // define output fuzzy sets an variables

        fl::OutputVariable* change_LOA = new fl::OutputVariable;
        change_LOA->setName("change_LOA");
        change_LOA->setDescription("");
        change_LOA->setEnabled(true);
        change_LOA->setRange(-1.000, 1.000);
        change_LOA->setLockValueInRange(false);
        change_LOA->setAggregation(new fl::Maximum);
        change_LOA->setDefuzzifier(new fl::LargestOfMaximum(200));
        change_LOA->setDefaultValue(fl::nan);
        change_LOA->setLockPreviousValue(false);
        change_LOA->addTerm(new fl::Triangle("change", 0.000, 1.000, 1.000));
        change_LOA->addTerm(new fl::Triangle("no_change", -1.000, -1.000, 0.000));
        engine->addOutputVariable(change_LOA);


//        fl::OutputVariable* outputVariable = new fl::OutputVariable;
//        outputVariable->setEnabled(true);
//        outputVariable->setName("change_LOA");
//        outputVariable->setRange(-1.000, 1.000);
//        outputVariable->fuzzyOutput()->setAggregation(new fl::Maximum); // the old setAccumulation is the new Aggregation?
//        outputVariable->setDefuzzifier(new fl::LargestOfMaximum(200));
//        outputVariable->setDefaultValue(fl::nan);

//        outputVariable->addTerm(new fl::Triangle("change", 0.000, 1.000, 1.000));
//        outputVariable->addTerm(new fl::Triangle("no_change", -1.000, -1.000, 0.000));
//        engine->addOutputVariable(outputVariable);

        // define fuzzy rule base

        fl::RuleBlock* ruleBlock = new fl::RuleBlock;
        ruleBlock->setName("");
        ruleBlock->setDescription("");
        ruleBlock->setEnabled(true);
        ruleBlock->setConjunction(new fl::Minimum);
        ruleBlock->setDisjunction(new fl::Maximum);
        ruleBlock->setImplication(new fl::Minimum);
// new rulebase ijrr + cogn av
        ruleBlock->setActivation(new fl::First(1, 0.000));
        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is unavailable and current_loa is teleop then change_LOA is change", engine));
        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is unavailable and current_loa is auto then change_LOA is no_change", engine));
        ruleBlock->addRule(fl::Rule::parse("if error is small or error is medium then change_LOA is no_change", engine));
        ruleBlock->addRule(fl::Rule::parse("if error is very large and speed is not reverse then change_LOA is change", engine));
        ruleBlock->addRule(fl::Rule::parse("if speed is reverse and error is large then change_LOA is no_change", engine));








// cimbination rulebase
//      ruleBlock->setActivation(new fl::General);
//      ruleBlock->addRule(fl::Rule::parse("if error is small and speed is reverse and current_loa is teleop and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is reverse and current_loa is teleop and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is reverse and current_loa is auto and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is reverse and current_loa is auto and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is zero and current_loa is teleop and cognitive_availability is unavailable then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is zero and current_loa is teleop and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is zero and current_loa is auto and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is zero and current_loa is auto and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is forward and current_loa is teleop and cognitive_availability is unavailable then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is forward and current_loa is teleop and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is forward and current_loa is auto and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is small and speed is forward and current_loa is auto and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is reverse and current_loa is teleop and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is reverse and current_loa is teleop and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is reverse and current_loa is auto and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is reverse and current_loa is auto and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is zero and current_loa is teleop and cognitive_availability is unavailable then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is zero and current_loa is teleop and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is zero and current_loa is auto and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is zero and current_loa is auto and cognitive_availability is available then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is forward and current_loa is teleop and cognitive_availability is unavailable then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is forward and current_loa is teleop and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is forward and current_loa is auto and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is medium and speed is forward and current_loa is auto and cognitive_availability is available then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is reverse and current_loa is teleop and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is reverse and current_loa is teleop and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is reverse and current_loa is auto and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is reverse and current_loa is auto and cognitive_availability is available then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is zero and current_loa is teleop and cognitive_availability is unavailable then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is zero and current_loa is teleop and cognitive_availability is available then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is zero and current_loa is auto and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is zero and current_loa is auto and cognitive_availability is available then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is forward and current_loa is teleop and cognitive_availability is unavailable then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is forward and current_loa is teleop and cognitive_availability is available then change_LOA is change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is forward and current_loa is auto and cognitive_availability is unavailable then change_LOA is no_change", engine));
//	ruleBlock->addRule(fl::Rule::parse("if error is large and speed is forward and current_loa is auto and cognitive_availability is available then change_LOA is change", engine));
      engine->addRuleBlock(ruleBlock);

//previous rule base -- without all combinations 
//        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is unavailable and current_loa is teleop then change_LOA is change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is unavailable and current_loa is auto then change_LOA is no_change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is available and current_loa is auto and error is large then change_LOA is change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is available and current_loa is teleop and error is small then change_LOA is no_change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is available and current_loa is teleop and error is medium then change_LOA is no_change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is available and current_loa is teleop and error is large and speed is forward then change_LOA is change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is available and current_loa is teleop and error is large and speed is zero then change_LOA is change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is available and current_loa is teleop and speed is reverse and error is large then change_LOA is no_change", engine));



//        fl::RuleBlock* ruleBlock = new fl::RuleBlock;
//        ruleBlock->setEnabled(true);
//        ruleBlock->setName("");
//        ruleBlock->setConjunction(new fl::Minimum);
//        ruleBlock->setDisjunction(new fl::Maximum);
//        ruleBlock->setImplication(new fl::Minimum);
//        // ruleBlock->setActivation(new fl::Threshold);This is now called implication operator
//        ruleBlock->addRule(fl::Rule::parse("if error is small or error is medium then change_LOA is no_change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if error is large and speed is not reverse then change_LOA is change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if speed is reverse and error is large then change_LOA is no_change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is not_available then change_LOA is change", engine));
//        ruleBlock->addRule(fl::Rule::parse("if cognitive_availability is available then change_LOA is no_change", engine));
//        engine->addRuleBlock(ruleBlock);

        //-------------------------------------------------------------------///


        ros::init(argc, argv, "mixed_initiative_controller");
        MixedInitiativeControllerDeepgaze controller_obj(engine);

        ros::Rate r(10); // 10 hz
        while (ros::ok())
        {
                ros::spinOnce();
                r.sleep();
        }

}
