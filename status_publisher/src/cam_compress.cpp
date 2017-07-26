
#include <ros/ros.h>
#include <image_transport/image_transport.h>


class CamCompress
{
public:

    CamCompress(): it_(nh_)
    {
        sub_ = it_.subscribe("/robot_cam/image", 1, &CamCompress::imageCallback, this);
        pub_ = it_.advertise("cam_compress/image", 1);
    }

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_ ;
    image_transport::Publisher pub_ ;
    image_transport::Subscriber sub_ ;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};


void CamCompress::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
pub_.publish(msg);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_compress");
    CamCompress camCompress;
    ros::spin();

}


