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
        // Initialise the noise period and the rectacular area of noise bounded by the rectacle of which the 4 points are the corners.
        private_nh.param("noise_period", noise_period_, 30.0);
        private_nh.param("x_max", x_max_, 532.61);
        private_nh.param("x_min", x_min_, 531.10);
        private_nh.param("y_max", y_max_, 48.41);
        private_nh.param("y_min", y_min_, 41.24);

        //randomGen_.seed(time(NULL)); // seed the generator
        laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("scan", 20, &LaserNoise::laserReadCallBAck, this);
        pose_sub_ = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &LaserNoise::poseCallback, this);
        reset_node_sub_ = n_.subscribe<std_msgs::Bool>("/noise_reset", 20, &LaserNoise::resetCallBack, this);

        scan_pub_ = n_.advertise<sensor_msgs::LaserScan>("scan_with_noise", 20);
        noise_active_pub_ = n_.advertise<std_msgs::Bool>("noise_active", 20);

        timerNoise_ = n_.createTimer(ros::Duration(noise_period_) , &LaserNoise::timerNoiseCallback, this, false, false);

        areaTriger_ = 0, timerTriger_ =0, timerActivated_= 0;
    }

private:

    //  boost::mt19937 randomGen_;

    ros::NodeHandle n_;
    ros::Subscriber laser_sub_ , pose_sub_ , reset_node_sub_;
    ros::Publisher scan_pub_ , noise_active_pub_;
    sensor_msgs::LaserScan addedNoiseScan_;
    ros::Timer timerNoise_ ;

    void laserReadCallBAck(const sensor_msgs::LaserScan::ConstPtr& msg);
    double GaussianKernel(double mu,double sigma), uniformNoise_;
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void timerNoiseCallback(const ros::TimerEvent&);
    void resetCallBack(const std_msgs::Bool::ConstPtr& msg);

    bool areaTriger_ , timerTriger_, timerActivated_;
    double noise_period_, x_max_, x_min_, y_max_, y_min_ ;

};


void LaserNoise::resetCallBack(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == true)
    {
        timerTriger_ = 0;
        timerNoise_.stop(); // needed to make sure the timer is not running in background
    }

}

void LaserNoise::laserReadCallBAck(const sensor_msgs::LaserScan::ConstPtr& msg)

{

    double sigma;
    double oldRange;
    std_msgs::Bool noise;
    addedNoiseScan_ = *msg ;


    // Guassian noise added
    if (areaTriger_ == 1 && timerTriger_ == 0)
    {
        for (int i=0; i < addedNoiseScan_.ranges.size() ; i++)

        {
            sigma = addedNoiseScan_.ranges[i] * 0.2; // Proportional standard deviation
            oldRange = addedNoiseScan_.ranges[i] ;
            addedNoiseScan_.ranges[i] = addedNoiseScan_.ranges[i] + GaussianKernel(0,sigma);

            if (addedNoiseScan_.ranges[i] > addedNoiseScan_.range_max)
            { addedNoiseScan_.ranges[i] = addedNoiseScan_.range_max;}

            else if (addedNoiseScan_.ranges[i] < addedNoiseScan_.range_min)
            { addedNoiseScan_.ranges[i] = oldRange;}
        }

        noise.data = true;
        noise_active_pub_.publish(noise);
    }


    addedNoiseScan_.header.stamp = ros::Time::now();

    scan_pub_.publish(addedNoiseScan_);


}

void LaserNoise::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)

{

    if ( (msg->pose.pose.position.x < x_max_) && (msg->pose.pose.position.x > x_min_)
         && (msg->pose.pose.position.y < y_max_) && (msg->pose.pose.position.y > y_min_) )

    {
        areaTriger_ =1;
        timerNoise_.start();
    }
    else
    {

        areaTriger_ = 0;
        std_msgs::Bool noise;
        noise.data = false;
        noise_active_pub_.publish(noise);
        //timerTriger_ = 0;
    }
}


void LaserNoise::timerNoiseCallback(const ros::TimerEvent&)
{
    timerTriger_ = 1;
    timerNoise_.stop();

    ROS_INFO("TIMER ACTIVATED");

}

// Utility for adding noise
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
