/*!
 * mixed_initiative_teleop_node.cpp
 * Copyright (c) 2014, Manolis Chiou
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*!

   @mainpage
   Joystick teleoperation node for use within the mixed initiative framework. The user can operate the robot in
   teleoperation mode and change on the fly autonomy level/mode. Also a stop button is implimented.
   It was ment to be used with an Xbox 360 joystick but should work with any joystick.
   <hr>

   @section usage Usage
   @par    After start roscore, you need load robot configuration file to parameter server first.
        For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
        then run drrobot_player first. then start ros joy node.
   @verbatim
   $ mixed_initiative_teleop
   @endverbatim

   <hr>
   @section topic ROS topics

   Publishes to (name / type):
   -@b /teleop/cmd_vel: will publish to /teleop/cmd_vel a geometry_msgs/Twist.msg type message to drrobot_player.
   For differential robots, linear.x is forward/backward speed (m/sec), and angular.z (rad/sec)is the angular speed.
   <hr>
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

class JoystickTeleop
{

public:
        JoystickTeleop();

private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
        void publish();

        ros::NodeHandle ph_, nh_;

        int linear_axis_, angular_axis_, control_button_, stop_button_, auto_button_, teleop_button_;
        double linear_scaling_, angular_scaling_;
        geometry_msgs::Twist last_msg_published_;
        boost::mutex publish_mutex_;

        ros::Publisher vel_pub_, loa_pub_, ai_loa_pub_;
        ros::Subscriber joy_sub_;
        ros::Timer timer_;
};

JoystickTeleop::JoystickTeleop() : ph_("~")

{
        // Default movement axis
        ph_.param("axis_linear", linear_axis_, 1);
        ph_.param("axis_angular", angular_axis_, 0);

        // Default scaling parameters
        ph_.param("scale_angular", angular_scaling_, 0.60);
        ph_.param("scale_linear", linear_scaling_, 0.4);

        //Default buttons for Xbox 360 joystick.
        ph_.param("teleop_button", teleop_button_, 3); // Y button
        ph_.param("stop_button", stop_button_, 4);     // LB button
        ph_.param("auto_button", auto_button_, 0);     // A button

        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/teleop/cmd_vel", 5);
        loa_pub_ = nh_.advertise<std_msgs::Int8>("/loa", 5);                 // publishing operators LOA choice
        ai_loa_pub_ = nh_.advertise<std_msgs::Int8>("/ai_suggested_loa", 5); // publishing the emulated AI's LOA choice

        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 2, &JoystickTeleop::joyCallback, this);

        timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&JoystickTeleop::publish, this));
}

void JoystickTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
        geometry_msgs::Twist cmd_vel;
        std_msgs::Int8 mode;
        std_msgs::Int8 ai_emulated_loa_;

        // movement commands
        cmd_vel.linear.x = linear_scaling_ * joy->axes[linear_axis_];
        cmd_vel.angular.z = angular_scaling_ * joy->axes[angular_axis_];
        last_msg_published_ = cmd_vel;

        // LOA choice/suggestion from operator
        if (joy->buttons[stop_button_])
        {
                mode.data = 0;
                loa_pub_.publish(mode);
        }
        if (joy->buttons[teleop_button_])
        {
                mode.data = 1;
                loa_pub_.publish(mode);
        }
        if (joy->buttons[auto_button_])
        {
                mode.data = 2;
                loa_pub_.publish(mode);
        }

        // LOA choice/suggestion from joystick but emulating the AI LOA suggested LOA switchies.
        // works with the digital "cross" in joystick
        if (joy->axes[7] == -1)
        {
                ai_emulated_loa_.data = 2;
                ai_loa_pub_.publish(ai_emulated_loa_);
        }
        if (joy->axes[7] == 1)
        {
                ai_emulated_loa_.data = 1;
                ai_loa_pub_.publish(ai_emulated_loa_);
        }
}

void JoystickTeleop::publish()
{
        boost::mutex::scoped_lock lock(publish_mutex_);
        vel_pub_.publish(last_msg_published_);
}

// the main function
int main(int argc, char **argv)
{
        ros::init(argc, argv, "mixed_initiative_teleop");
        JoystickTeleop joystick_teleop;
        ros::spin();
}
