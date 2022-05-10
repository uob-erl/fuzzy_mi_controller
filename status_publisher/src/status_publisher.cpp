
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/package.h>
#include <string.h>



class StatusPublisher
{
public:

StatusPublisher();

private:

ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Publisher loa_pub_, nav_status_pub_;
ros::Subscriber loa_sub_, nav_status_sub_, nav_result_sub_ ;
ros::Timer timerPubStatus_;

int nav_status_, nav_result_;
std::string pathTeleop_, pathAuto_,pathStop_, pathCanceled_;
std::string pathActive_, pathSucceeded_,pathAborted_;
cv_bridge::CvImage cvAuto_, cvTeleop_, cvStop_, cvCanceled_;       // intermediate cv_bridge images
cv_bridge::CvImage cvActive_, cvSucceeded_, cvAborted_;
sensor_msgs::Image rosImgAuto_, rosImgTeleop_, rosImgStop_, rosImgCanceled_;
     // ROS msg images
sensor_msgs::Image rosImgActive_, rosImgSucceeded_, rosImgAborted_;

void loaCallBack(const std_msgs::Int8::ConstPtr& loa);
void nav_statusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& nav_status);
void nav_resultCallBack(const move_base_msgs::MoveBaseActionResult::ConstPtr& nav_result);
void timerPubStatusCallback(const ros::TimerEvent&);


};


// Constractor
StatusPublisher::StatusPublisher() : it_(nh_)
{
        // Initialise status/result values

        nav_status_ = -1;
        nav_result_ = -1;

        // Subscribers
        loa_sub_ = nh_.subscribe<std_msgs::Int8>("/loa", 1, &StatusPublisher::loaCallBack, this);
        nav_status_sub_  = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1,
                                                                          &StatusPublisher::nav_statusCallBack, this);
        nav_result_sub_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1,
                                                                              &StatusPublisher::nav_resultCallBack,this);

        // publishers
        loa_pub_ = it_.advertise("/robot_status/loa", 1, true);
        nav_status_pub_ = it_.advertise("/robot_status/nav",1, true);
        timerPubStatus_ = nh_.createTimer(ros::Duration(0.100), &StatusPublisher::timerPubStatusCallback, this);

        // Path where the images are
        pathTeleop_ = ros::package::getPath("status_publisher");
        pathTeleop_.append("/images/teleop.png");

        pathAuto_ = ros::package::getPath("status_publisher");
        pathAuto_.append("/images/auto.png");

        pathStop_ = ros::package::getPath("status_publisher");
        pathStop_.append("/images/stop.png");

        pathActive_ = ros::package::getPath("status_publisher");
        pathActive_.append("/images/active.png");

        pathSucceeded_ = ros::package::getPath("status_publisher");
        pathSucceeded_.append("/images/succeeded.png");

        pathAborted_ = ros::package::getPath("status_publisher");
        pathAborted_.append("/images/aborted.png");

   
        pathCanceled_ = ros::package::getPath("status_publisher");
        pathCanceled_.append("/images/canceled.png");


        // Safety Check if actually the image is there and loaded
        if( cv::imread(pathTeleop_.c_str()).empty() )
                ROS_FATAL("Teleop image was not loaded. Could not be found on %s", pathTeleop_.c_str());

        else if (cv::imread(pathAuto_.c_str()).empty() )
                ROS_FATAL("Auto image was not loaded. Could not be found on %s", pathAuto_.c_str());

        else if (cv::imread(pathStop_.c_str()).empty() )
                ROS_FATAL("Stop image was not loaded. Could not be found on %s", pathStop_.c_str());

        else if (cv::imread(pathActive_.c_str()).empty()  )
                ROS_FATAL("Active image was not loaded. Could not be found on %s", pathActive_.c_str());

        else if ( cv::imread(pathSucceeded_.c_str()).empty() )
                ROS_FATAL("Succeded image was not loaded. Could not be found on %s", pathSucceeded_.c_str());

        else if ( cv::imread(pathAborted_.c_str()).empty() )
                ROS_FATAL("Aborted image was not loaded. Could not be found on %s", pathAborted_.c_str());


        else if ( cv::imread(pathCanceled_.c_str()).empty() )
                ROS_FATAL("Goal cancel image was not loaded. Could not be found on %s", pathCanceled_.c_str());

        else
                ROS_INFO("All images loaded successfuly");


        // Load auto image with openCv
        cvAuto_.image = cv::imread(pathAuto_.c_str());
        cvAuto_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load teleop image with openCv
        cvTeleop_.image = cv::imread(pathTeleop_.c_str());
        cvTeleop_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Stop image with openCv
        cvStop_.image = cv::imread(pathStop_.c_str());
        cvStop_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Active image with openCv
        cvActive_.image = cv::imread(pathActive_.c_str());
        cvActive_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Succeeded image with openCv
        cvSucceeded_.image = cv::imread(pathSucceeded_.c_str());
        cvSucceeded_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Aborted image with openCv
        cvAborted_.image = cv::imread(pathAborted_.c_str());
        cvAborted_.encoding = sensor_msgs::image_encodings::BGR8;
      
       
        // Load Cancel goal image with openCv
        cvCanceled_.image = cv::imread(pathCanceled_.c_str());
        cvCanceled_.encoding = sensor_msgs::image_encodings::BGR8;

        // convert to ROS image type
        cvAuto_.toImageMsg(rosImgAuto_);
        cvTeleop_.toImageMsg(rosImgTeleop_);
        cvStop_.toImageMsg(rosImgStop_);
        cvActive_.toImageMsg(rosImgActive_);
        cvSucceeded_.toImageMsg(rosImgSucceeded_);
        cvAborted_.toImageMsg(rosImgAborted_);
        cvCanceled_.toImageMsg(rosImgCanceled_);

        // Publish the default mode
        loa_pub_.publish(rosImgStop_);
        nav_status_pub_.publish(rosImgStop_);
}



// takes care of loa publising in rviz
void StatusPublisher::loaCallBack(const std_msgs::Int8::ConstPtr& mode)
{
        if (mode->data == 0)
                loa_pub_.publish(rosImgStop_);
        if (mode->data == 1)
                loa_pub_.publish(rosImgTeleop_);
        if (mode->data == 2)
                loa_pub_.publish(rosImgAuto_);

}



// takes care of NAVIGATION current STATUS
void StatusPublisher::nav_statusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& nav_status)
{
        if (!nav_status->status_list.empty()) //First make you the vector is not empty to avoid memory allocation errors.
        {

                if (nav_status->status_list.size() > 1)
                {
                        actionlib_msgs::GoalStatus goalStatus = nav_status->status_list[1];
                        nav_status_ = goalStatus.status;
                }
                else
                {
                        actionlib_msgs::GoalStatus goalStatus = nav_status->status_list[0];
                        nav_status_ = goalStatus.status;
                }

        }
}


// takes care of NAVIGATION end Result
void StatusPublisher::nav_resultCallBack(const move_base_msgs::MoveBaseActionResult::ConstPtr& nav_result)
{
        nav_result_ = nav_result->status.status;
}


// The timer function that fuses everyting and publishes to the interface
void StatusPublisher::timerPubStatusCallback(const ros::TimerEvent&)
{
        // Publish current/running state
        if (nav_status_ == 1)
                nav_status_pub_.publish(rosImgActive_);


        // Publish end state of goals


        else if (nav_result_== 3)
        {
                nav_status_pub_.publish(rosImgSucceeded_);
                nav_result_ = -1;
        }

        else if (nav_result_== 2)
        {
                nav_status_pub_.publish(rosImgCanceled_);
                nav_result_ = -1;
        }

        else if (nav_result_ == 4 )
        {
                nav_status_pub_.publish(rosImgAborted_);
                nav_result_ = -1;
        }


        else {
                //ROS_INFO("Status Something else?? Check /move_base/status for code");
        }


}

int main(int argc, char** argv)
{

        ros::init(argc, argv, "status_publisher");

        StatusPublisher publishStatus;


        ros::Rate loop_rate(10);

        // The main Loop where everything is runing

        while (ros::ok())
        {
                ros::spinOnce();
                loop_rate.sleep();
        }


        return 0;
}

