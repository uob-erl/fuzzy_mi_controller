# Important note
Due to some updates certain experiments such as the negotiation require a specific commit in the erl Husky repo. in order to run.
The problem seems to come from the melodic updates that remove kinect for RealSense and some other changes Clearpath made. We need to update these at some point.

For now:
`git checkout 147f4e69b481f171f6e6fc56a873a6d98419c170` in the ERL's husky repo Melodic branch. 
Then `sudo apt-get install ros-melodic-realsense2-description` . 

# Husky Mixed-Initative control
Repository for the Mixed-Inititive control experiments with robots (real or simulated). If you use any of these packages please cite the following paper or one of the papers mentioned in the specific branch.

- Chiou, M., Hawes, N., & Stolkin, R. (2021). Mixed-initiative Variable Autonomy for Remotely Operated Mobile Robots. ACM Transactions on Human-Robot Interaction, 10(4), 1–34. https://doi.org/10.1145/3472206

 - For code and experiments in the ICHMS 2020 paper "Human operator cognitive availability aware Mixed-Initiative control" please use the "cognitive_availability" branch.
 
The package "experiments_launch" contains all the top level .launch files. It is a good start if you want to understand how stuff work. In order to work, requires the ERL's forked package of husky drivers: https://github.com/uob-erl/husky_learning_exp . Also it requires to clone and use the newest nav stack for kinetic: https://github.com/ros-planning/navigation/tree/kinetic-devel.

# Installing ROS dependencies

Most ROS dependancies should be taken care of by `rosdep install --from-paths src --ignore-src -r -y` from your catkin workspace. If not:

```sh
sudo apt-get install ros-kinetic-gmapping
sudo apt-get install ros-kinetic-robot-localization
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-kinetic-audio-common
sudo apt-get install ros-kinetic-frontier-exploration
sudo apt-get install ros-kinetic-hector-slam
sudo apt-get install ros-kinetic-p2os-urdf
```

# Installing fuzzylite 6 cpp library dependency
For the controller to compile and run, the fuzzylite library is needed. The controller uses version 6 of fuzzylite. Please use the instructions below to install fuzzylite 6. 

1) Once download from https://fuzzylite.com/ , do a `cd` into the fuzzylite folder.

2)  run `./build.sh release`

3) Use checkinstall to create a debian file instead of installing the library old school. It makes life easier e.g. you can install or remove library via apt-get.

```sh
sudo apt-get install checkinstall
cd release
sudo checkinstall --pkgname=fuzzylite6
```

Alternatively:
```sh
cd release
sudo make install
````

In order to unistall, if make install was used, `cat install_manifest.txt | xargs echo sudo rm | sh`.

4) The library is installed.

Some time you can get an error while running ROS with fuzzylite "cannot open shared object file X: No such file or directory":

To fix it: `sudo /sbin/ldconfig -v`
See this: https://itsfoss.com/solve-open-shared-object-file-quick-tip/


# Running simulated robot arena in Gazebo
Assuming ROS and Gazebo are correctly installed and working, you can run a simulated Husky robot in a Search and Rescue scenario using the Mixed Initiative (MI) controller. The buttons for switching Level of Autonomy and for controlling the robot in teleoperation are mapped for a xbox controller.

To run the simulated Husky and the MI control:
```sh
roslaunch experiments_launch husky_gazebo_mi_experiment.launch
roslaunch experiments_launch mi_control.launch
````
# Running real world experiment
Docs and code soon
