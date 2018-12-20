
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <sound_play/sound_play.h>








class StatusSoundPublisher
{
public:

StatusSoundPublisher();

private:

ros::NodeHandle nh_;
ros::Subscriber loa_sub_;
sound_play::SoundClient sound_client_;

void loaCallBack(const std_msgs::Int8::ConstPtr& mode);

};


// Constractor
StatusSoundPublisher::StatusSoundPublisher()
{
        // Subscribers
        loa_sub_ = nh_.subscribe<std_msgs::Int8>("/loa", 1, &StatusSoundPublisher::loaCallBack, this);
}

// takes care of LOA
void StatusSoundPublisher::loaCallBack(const std_msgs::Int8::ConstPtr& mode)
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

        ros::init(argc, argv, "status_sound_publisher");

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
