# Husky Mixed-Initative control
Repository for the Mixed-Inititive control experiments with Husky robot. For older experiments (IJRR paper) see ijrr branch.
The package "experiments_launch" contains all the top level .launch files. It is a good start if you want to understand how stuff work.

# Installing ROS dependencies
```sh
sudo apt-get install ros-kinetic-gmapping
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-audio-common
sudo apt-get install ros-kinetic-frontier-exploration
sudo apt-get install ros-kinetic-hector-slam
sudo apt-get install ros-kinetic-p2os-urdf
```

# Installing fuzzylite 6 cpp library
The original fuzzy controller was made using the version 4.0 of fuzzylite library which is now unavailable. Thus fuzzylite 6.0 can be used. WARNING: the fuzzy controller to be compiled with version 6 of fuzzylite was not tested in practice. Some code changes were neccessery for it to compile, hence the behavior might have changed (unlikely though!).
1) Do a `cd` into the fuzzylite folder.

2)  run `./build.sh release`

3) Use checkinstall to create a debian file instead of installing the library old school. It makes life easier e.g. you can install or remove library via apt-get.

```sh
sudo apt-get install checkinstall
cd release
sudo checkinstall --pkgname=fuzzylight6
```

Alternatively:
```sh
cd release
sudo make install
````

In order to unistall, if make install was used, `cat install_manifest.txt | xargs echo sudo rm | sh`.

4) The library is installed.

# Running simulated robot arena in Gazebo
Assuming ROS and Gazebo are correctly installed and working, you can run a simulated Husky robot in a Search and Rescue scenario using the Mixed Initiative (MI) controller. The buttons for switching Level of Autonomy and for controlling the robot in teleoperation are mapped for a xbox controller.

To run the simulated Husky and the MI control:
```sh
roslaunch experiments_launch husky_gazebo_mi_experiment.launch
roslaunch experiments_launch mi_control.launch
````
# Running real world experiment
Docs and code soon
