
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <sound_play/sound_play.h>








class StatusSoundPublisher
{
public:

    StatusSoundPublisher();


private:

    ros::NodeHandle nh_;
    ros::Subscriber mode_sub_ ;
    sound_play::SoundClient sound_client_ ;


    void modeCallBack(const std_msgs::Int8::ConstPtr& mode);


};


// Constractor
StatusSoundPublisher::StatusSoundPublisher()
{



    // Subscribers
    mode_sub_ = nh_.subscribe<std_msgs::Int8>("/control_mode", 1 , &StatusSoundPublisher::modeCallBack, this);



}





// takes care of MODE
void StatusSoundPublisher::modeCallBack(const std_msgs::Int8::ConstPtr& mode)
{
    if (mode->data == 0)
    {
        sound_client_.say("stoped");
    }

    else if (mode->data == 1)
    {
        sound_client_.playWaveFromPkg("variable_autonomy","auto_pilot.wav");
        ros::Duration(1.2).sleep();
        sound_client_.say("teleoperation");
    }

    else if (mode->data == 2)
    {
        sound_client_.playWaveFromPkg("variable_autonomy","auto_pilot.wav");
        ros::Duration(1.2).sleep();
        sound_client_.say("autonomy");
    }


}






int main(int argc, char** argv)
{

    ros::init(argc, argv, "status_publisher");

    StatusSoundPublisher publish_sound_status;


    ros::Rate loop_rate(10);

    // The main Loop where everything is runing

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
