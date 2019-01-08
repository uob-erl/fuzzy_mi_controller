#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class LaserNoise
{
public:
LaserNoise()
{
        ros::NodeHandle private_nh("laser_noise");
        // Initialise the noise period...
        private_nh.param("noise_period", noise_period_, 30.0);
        // and the rectacular area of noise bounded by the rectacle of which the 4 points are the corners.
        private_nh.param("x_max", x_max_, 532.61);
        private_nh.param("x_min", x_min_, 531.10);
        private_nh.param("y_max", y_max_, 48.41);
        private_nh.param("y_min", y_min_, 41.24);

        //randomGen_.seed(time(NULL)); // seed the generator
        laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("scan", 20, &LaserNoise::laserReadCallBAck, this);
        pose_sub_ = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 20, &LaserNoise::poseCallback, this);
        reset_node_sub_ = n_.subscribe<std_msgs::Bool>("/laser_noise_reset", 20, &LaserNoise::resetCallBack, this);
        joy_triggered_noise_sub_ = n_.subscribe<std_msgs::Bool>("/joy_triggered_noise", 20, &LaserNoise::joyNoiseCallBack, this);

        scan_with_noise_pub_ = n_.advertise<sensor_msgs::LaserScan>("scan_with_noise", 20);
        laser_noise_active_pub_ = n_.advertise<std_msgs::Bool>("laser_noise_active", 20);

        timer_noise_ = n_.createTimer(ros::Duration(noise_period_), &LaserNoise::timerNoiseCallback, this, false, false);

        area_trigger_ = 0, timer_trigger_ =0, timer_activated_= 0, joy_noise_trigger_ = 0;
        noise_scale_ = 0.3;
}

private:

//  boost::mt19937 randomGen_;

ros::NodeHandle n_;
ros::Subscriber laser_sub_, pose_sub_, reset_node_sub_, joy_triggered_noise_sub_;
ros::Publisher scan_with_noise_pub_, laser_noise_active_pub_;
ros::Timer timer_noise_;

void laserReadCallBAck(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
double GaussianKernel(double mu,double sigma), uniformNoise_;
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void timerNoiseCallback(const ros::TimerEvent&);
void resetCallBack(const std_msgs::Bool::ConstPtr& msg);
void joyNoiseCallBack(const std_msgs::Bool::ConstPtr& msg);

bool area_trigger_, timer_trigger_, timer_activated_, joy_noise_trigger_;
double noise_period_, noise_scale_, x_max_, x_min_, y_max_, y_min_;

};

//
void LaserNoise::resetCallBack(const std_msgs::Bool::ConstPtr& msg)
{
        if (msg->data == true)
        {
                timer_trigger_ = 0;
                timer_noise_.stop(); // needed to make sure the timer is not running in background
        }

}

// reads published by laser scans and adds noise to them
void LaserNoise::laserReadCallBAck(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
        double sigma;
        double old_range;
        std_msgs::Bool noise;
        sensor_msgs::LaserScan laser_scan = *scan_msg;

        // Guassian noise added
        if ( (area_trigger_ == 1 || joy_noise_trigger_ == 1) && timer_trigger_ == 0)
        {
                for (int i=0; i < laser_scan.ranges.size(); i++)
                {
                        sigma = laser_scan.ranges[i] * noise_scale_; // Proportional standard deviation
                        old_range = laser_scan.ranges[i];
                        laser_scan.ranges[i] = laser_scan.ranges[i] + GaussianKernel(0,sigma);

                        if (laser_scan.ranges[i] > laser_scan.range_max)
                        { laser_scan.ranges[i] = laser_scan.range_max; }

                        else if (laser_scan.ranges[i] < laser_scan.range_min)
                        { laser_scan.ranges[i] = old_range; }
                }
                noise.data = true;
                laser_noise_active_pub_.publish(noise);
        }
        else
        {
                noise.data = false;
                laser_noise_active_pub_.publish(noise);
        }
        laser_scan.header.stamp = ros::Time::now();
        scan_with_noise_pub_.publish(laser_scan);
}

// reads robot pose and if robot is inside a predifined area, activates noise trigger
void LaserNoise::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
        // if ( (msg->pose.pose.position.x < x_max_) && (msg->pose.pose.position.x > x_min_)
        //      && (msg->pose.pose.position.y < y_max_) && (msg->pose.pose.position.y > y_min_) )
        //
        // {
        //         area_trigger_ =1;
        //         timer_noise_.start();
        // }
        // else
        // {
        //         area_trigger_ = 0;
        //         std_msgs::Bool noise;
        //         noise.data = false;
        //         laser_noise_active_pub_.publish(noise);
        //         //timer_trigger_ = 0;
        // }
}

// reads robot pose and if robot is inside a predifined area, activates noise trigger
void LaserNoise::joyNoiseCallBack(const std_msgs::Bool::ConstPtr& msg)
{
        if (msg->data == true)
        {
                joy_noise_trigger_ = 1;
                timer_noise_.start();
        }
        else
        {
                joy_noise_trigger_ = 0;
                std_msgs::Bool noise;
                noise.data = false;
                laser_noise_active_pub_.publish(noise);
        }
}

// this is the timer deciding for how long the noise will be activated
void LaserNoise::timerNoiseCallback(const ros::TimerEvent&)
{
        timer_trigger_ = 1;
        timer_noise_.stop();
        ROS_INFO("TIMER ACTIVATED");
}

// Utility function for adding Guassian noise
double LaserNoise::GaussianKernel(double mu,double sigma)
{
        // using Box-Muller transform to generate two independent standard normally disbributed normal variables

        double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
        double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
        double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
        //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
        // we'll just use X
        // scale to our mu and sigma
        X = sigma * X + mu;
        return X;
}


int main(int argc, char** argv)
{
        ros::init(argc, argv, "laser_noise");
        LaserNoise lasernoise;

        ros::spin();
}
