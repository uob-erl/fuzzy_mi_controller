#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <mysim> environment

Feel free to edit this template as you like!
"""

from morse.builder import *


# 'morse add robot <name> mysim' can help you to build custom robots.
robot = Pioneer3DX()
robot.properties(GroundRobot = True)

# second cam robot
#robot_cam = Pioneer3DX()
#robot_cam.properties(GroundRobot = True)
#robot_cam.translate(-3.2, 7, 0.0)
#robot_cam.rotate(0, 0, 0)

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html

# place your component at the correct location
# pioneer3dx.rotate(<rx>, <ry>, <rz>)
#robot.translate(1, 0, 0.0) <- training
robot.translate(-3.9, 6.5, 0.0)
robot.rotate(0, 0, 1.57)

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

# on board Camera
camera = VideoCamera()
camera.translate(-0.20, 0.0, 0.4)
camera.rotate(0, -0.35, 0.0)
camera.properties(cam_width =640, cam_height=360)
robot.append(camera)
camera.add_stream('ros', topic = '/robot_cam')

# Camera 2
#camera2 = VideoCamera()
#camera2.translate(0, 0.0, 1)
#camera2.rotate(0, -1.56, 0.0)
#camera2.properties(cam_width =640, cam_height=360)
#robot_cam.append(camera2)
#camera2.add_stream('ros', topic = '/cam1')


# set 'fastmode' to True to switch to wireframe mode
# 'tum_kitchen/tum_kitchen' '/home/manolis/Dropbox/MORSE/maze_new' /home/manolis/Dropbox/MORSE/training_arena/training_arena
env = Environment('/home/manolis/hri_ws/src/fuzzy_mi_controller_repo/experiments_launch/world/exp2_arena/new_arena.blend')
env.set_camera_location([10.0, -10.0, 10.0])
env.set_camera_rotation([1.05, 0, 0.78])
