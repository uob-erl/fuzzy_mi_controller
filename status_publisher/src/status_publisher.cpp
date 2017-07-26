
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <frontier_exploration/ExploreTaskActionResult.h>
#include <ros/package.h>
#include <string.h>







class StatusSoundPublisher
{
public:

    StatusSoundPublisher();


private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher mode_pub_ , navStatus_pub_;
    ros::Subscriber mode_sub_ , navStatus_sub_ , navResult_sub_, explStatus_sub_ , explResult_sub_;
    ros::Timer timerPubStatus_;

    int explStatus_, navStatus_, explResult_, navResult_;
    std::string pathTeleop_, pathAuto_ ,pathStop_, pathCanceled_ , pathExplorationDone_;
    std::string pathActive_, pathSucceeded_ ,pathAborted_, pathExploring_;
    cv_bridge::CvImage cvAuto_, cvTeleop_ , cvStop_, cvCanceled_, cvExplorationDone_;  // intermediate cv_bridge images
    cv_bridge::CvImage cvActive_, cvSucceeded_, cvAborted_, cvExploring_;
    sensor_msgs::Image rosImgAuto_ , rosImgTeleop_ , rosImgStop_, rosImgCanceled_, rosImgExplorationDone_; // ROS msg images
    sensor_msgs::Image rosImgActive_, rosImgSucceeded_, rosImgAborted_, rosImgExploring_;
    void modeCallBack(const std_msgs::Int8::ConstPtr& mode);
    void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& navStatus);
    void navResultCallBack(const move_base_msgs::MoveBaseActionResult::ConstPtr& navResult);
    void explStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& explStatus);
    void explResultCallback(const frontier_exploration::ExploreTaskActionResult::ConstPtr& explResult);
    void timerPubStatusCallback(const ros::TimerEvent&);


};


// Constractor
StatusSoundPublisher::StatusSoundPublisher(): it_(nh_)
{

    // Initialise status/result values
    explStatus_ = -1;
    explResult_ = -1;
    navStatus_ = -1;
    navResult_ = -1;


    // Subscribers
    mode_sub_ = nh_.subscribe<std_msgs::Int8>("/control_mode", 1 , &StatusSoundPublisher::modeCallBack, this);
    navStatus_sub_  = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1,
                                                                     &StatusSoundPublisher::navStatusCallBack, this);
    navResult_sub_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1,
                                                                         &StatusSoundPublisher::navResultCallBack,this);
    explStatus_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/explore_server/status",1,
                                                                     &StatusSoundPublisher::explStatusCallback, this);
    explResult_sub_ = nh_.subscribe<frontier_exploration::ExploreTaskActionResult>("/explore_server/result", 1,
                                                                                   &StatusSoundPublisher::explResultCallback,this);

    // publishers
    mode_pub_ = it_.advertise("/robot_status/mode", 1, true);
    navStatus_pub_ = it_.advertise("/robot_status/nav",1, true);
    timerPubStatus_ = nh_.createTimer(ros::Duration(0.100), &StatusSoundPublisher::timerPubStatusCallback, this);

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

    pathExploring_ = ros::package::getPath("status_publisher");
    pathExploring_.append("/images/exploring.png");

    pathExplorationDone_ = ros::package::getPath("status_publisher");
    pathExplorationDone_.append("/images/exp_done.png");

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

    else if ( cv::imread(pathExploring_.c_str()).empty() )
        ROS_FATAL("Exploring image was not loaded. Could not be found on %s", pathExploring_.c_str());

    else if ( cv::imread(pathExplorationDone_.c_str()).empty() )
        ROS_FATAL("Done exploratio image was not loaded. Could not be found on %s", pathExplorationDone_.c_str());

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
    // Load Exploring image with openCv
    cvExploring_.image = cv::imread(pathExploring_.c_str());
    cvExploring_.encoding = sensor_msgs::image_encodings::BGR8;
    // Load Exploration done image with openCv
    cvExplorationDone_.image = cv::imread(pathExplorationDone_.c_str());
    cvExplorationDone_.encoding = sensor_msgs::image_encodings::BGR8;
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
    cvExploring_.toImageMsg(rosImgExploring_);
    cvExplorationDone_.toImageMsg(rosImgExplorationDone_);
    cvCanceled_.toImageMsg(rosImgCanceled_);


    // Publish the default mode
    mode_pub_.publish(rosImgStop_);
    navStatus_pub_.publish(rosImgStop_);

}





// takes care of MODE
void StatusSoundPublisher::modeCallBack(const std_msgs::Int8::ConstPtr& mode)
{
    if (mode->data == 0)
        mode_pub_.publish(rosImgStop_);
    if (mode->data == 1)
        mode_pub_.publish(rosImgTeleop_);
    if (mode->data == 2)
        mode_pub_.publish(rosImgAuto_);

}


// takes care of EXPLORATION current STATUS
void StatusSoundPublisher::explStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& explStatus)

{


    if (!explStatus->status_list.empty()) //First make sure the vector is not empty to avoid memory allocation errors.
    {
        if (explStatus->status_list.size() > 1)
        {
            actionlib_msgs::GoalStatus goalStatus = explStatus->status_list[1];
            explStatus_ = goalStatus.status;
        }
        else
        {

            actionlib_msgs::GoalStatus goalStatus = explStatus->status_list[0];
            explStatus_ = goalStatus.status;
        }
    }
}

// takes care of EXPLORATION end Result

void StatusSoundPublisher::explResultCallback(const frontier_exploration::ExploreTaskActionResult::ConstPtr& explResult)
{
    explResult_ = explResult->status.status;
}



// takes care of NAVIGATION current STATUS
void StatusSoundPublisher::navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& navStatus)
{
    if (!navStatus->status_list.empty()) //First make you the vector is not empty to avoid memory allocation errors.
    {

        if (navStatus->status_list.size() > 1)
        {
            actionlib_msgs::GoalStatus goalStatus = navStatus->status_list[1];
            navStatus_ = goalStatus.status;
        }
        else
        {
            actionlib_msgs::GoalStatus goalStatus = navStatus->status_list[0];
            navStatus_ = goalStatus.status;
        }

    }
}


// takes care of NAVIGATION end Result
void StatusSoundPublisher::navResultCallBack(const move_base_msgs::MoveBaseActionResult::ConstPtr& navResult)
{
    navResult_ = navResult->status.status;
}


// The timer function that fuses everyting and publishes to the interface
void StatusSoundPublisher::timerPubStatusCallback(const ros::TimerEvent&)
{
    // Publish current/running state
    if (navStatus_ == 1 && explStatus_ != 1)
        navStatus_pub_.publish(rosImgActive_);

    else if (navStatus_ == 1 && explStatus_ == 1)
        navStatus_pub_.publish(rosImgExploring_);


    // Publish end state of goals
    else if (navResult_ == 3 && explResult_ == 4)
    {
        navStatus_pub_.publish(rosImgExplorationDone_);
        navResult_ = -1;  //setting flags back to default to avoud wrong state pub
        explResult_ = -1;
    }

    else if (navResult_== 2 && explResult_ == 3)
    {
        navStatus_pub_.publish(rosImgExplorationDone_);
        navResult_ = -1;
        explResult_ = -1;
    }

    else if (navResult_== 3 && explResult_ != 4)
    {
        navStatus_pub_.publish(rosImgSucceeded_);
        navResult_ = -1;
        explResult_ = -1;
    }

    else if (navResult_== 2 && explResult_ !=3 && explStatus_ != 1)
    {
        navStatus_pub_.publish(rosImgCanceled_);
        navResult_ = -1;
        explResult_ = -1;
    }

    else if (navResult_ == 4 )
    {
        navStatus_pub_.publish(rosImgAborted_);
        navResult_ = -1;
        explResult_ = -1;
    }


    else {
        //ROS_INFO("Status Something else?? Check /move_base/status for code");
    }


}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "status_publisher");

    StatusSoundPublisher publishStatus;


    ros::Rate loop_rate(10);

    // The main Loop where everything is runing

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
