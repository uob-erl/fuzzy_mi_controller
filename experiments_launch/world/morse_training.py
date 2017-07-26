
#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <mysim> environment

Feel free to edit this template as you like!
"""

from morse.builder import *


# 'morse add robot <name> mysim' can help you to build custom robots.
robot = Pioneer3DX()
robot.properties(GroundRobot = True)

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html

# place your component at the correct location
robot.translate(1, -1.65, 0.0)
robot.rotate(0, 0, -1.57)

# Add a motion controller
motion = MotionVWDiff()
robot.append(motion)
motion.add_stream('ros', topic = '/cmd_vel')




# Odometry
odometry = Odometry()
robot.append(odometry)
odometry.add_stream('ros', topic = '/odom', frame_id = '/odom' , child_frame_id = '/base_link')


# Laser sensor
hokuyo = Hokuyo()
hokuyo.frequency(40.0)
hokuyo.translate(x=-0.15, z=0.20)
robot.append(hokuyo)
hokuyo.add_stream('ros', topic = '/scan' , frame_id = '/base_laser')

# Camera
camera = VideoCamera()
camera.translate(-0.20, 0.0, 0.4)
camera.rotate(0, -0.35, 0.0)
camera.properties(cam_width =640, cam_height=360)
robot.append(camera)
camera.add_stream('ros', topic = '/morse_cam')


# set 'fastmode' to True to switch to wireframe mode
# 'tum_kitchen/tum_kitchen' '/home/manolis/Dropbox/MORSE/maze_new' /home/manolis/Dropbox/MORSE/training_arena/training_arena
env = Environment('/home/manolis/Dropbox/MORSE/training_arena/training_arena')
env.set_camera_location([10.0, -10.0, 10.0])
env.set_camera_rotation([1.05, 0, 0.78])




