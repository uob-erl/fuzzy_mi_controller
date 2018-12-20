
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>


class ExperimentLogger
{
public:
    ExperimentLogger()
    {
        experiment_init_pub_ = nh_.advertise<std_msgs::Bool>("/experiment_started", 1);
        secondary_task_init_pub_ = nh_.advertise<std_msgs::Bool>("/secondary_task/started", 1);
        secondary_task_end_pub_ = nh_.advertise<std_msgs::Bool>("/secondary_task/ended", 1);
        laser_noise_reset_pub_ = nh_.advertise<std_msgs::Bool>("/laser_noise_reset", 1);

        // joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 2 , &ExperimentLogger::joyPublishedCallBack,this);
        joy2_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy2", 2 , &ExperimentLogger::joy2PublishedCallBack,this);
        clearCostmapSrv_ = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps") ;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher experiment_init_pub_ , secondary_task_init_pub_, secondary_task_end_pub_, laser_noise_reset_pub_;
    ros::Subscriber joy_sub_ , joy2_sub_;
    ros::ServiceClient clearCostmapSrv_ ;

    // void joyPublishedCallBack(const sensor_msgs::Joy::ConstPtr& msg);
    void joy2PublishedCallBack(const sensor_msgs::Joy::ConstPtr& msg);
};


// void ExperimentLogger::joyPublishedCallBack(const sensor_msgs::Joy::ConstPtr& msg)
// {
//
// }

void ExperimentLogger::joy2PublishedCallBack(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (msg->buttons[7] == true) // denotes when the trials starts and when it finishes (START<!-- change this /jsX depending on your joystick port --> button)
    {
        std_msgs::Bool started;
        started.data = true;
        experiment_init_pub_.publish(started);
    }

    if (msg->buttons[2] == true) // denotes when the secondary task starts (X)
    {
        std_msgs::Bool started;
        started.data = true;
        secondary_task_init_pub_.publish(started);
    }

    if (msg->buttons[1] == true)  // denotes when the secondary task finishes (B)
    {
        std_msgs::Bool ended;
        ended.data = true;
        secondary_task_end_pub_.publish(ended);
    }

    if (msg->buttons[0] == true)  // resets sensor noise node so it can start again on the way back (A)
    {
        std_msgs::Bool reset;
        reset.data = true;
        laser_noise_reset_pub_.publish(reset);
    }

    if (msg->buttons[3] == true)  // Clears cost maps (Y)
    {
        std_srvs::Empty srv ;
        clearCostmapSrv_.call(srv);
    }

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "experiment_logger");
    ExperimentLogger experimentLogger;

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}
