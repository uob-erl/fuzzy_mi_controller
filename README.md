# Human operator intent aware Mixed-Initative control
Repository of the Human operator intent aware Mixed-Initative controller. The package "experiments_launch" contains all the top level .launch files. It is a good start if you want to understand how stuff work.

For the controller to work it requires the operator intent Bayesian recursive filter https://github.com/uob-erl/operator_intent_packages. Additionaly the experiments require the ERL's forked package of husky drivers: https://github.com/uob-erl/husky.git , branch "learning_effects_exp". Lastly, the experiments require the use of the newest nav stack for kinetic: https://github.com/ros-planning/navigation/tree/kinetic-devel.

# Installing ROS dependencies
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

# Running simulated robot arena in Gazebo
Assuming ROS and Gazebo are correctly installed and working, you can run a simulated Husky robot in a Search and Rescue scenario using the Mixed Initiative (MI) controller. The buttons for switching Level of Autonomy and for controlling the robot in teleoperation are mapped for a xbox controller.

To run the simulated Husky and the MI control:
```sh
roslaunch experiments_launch husky_gazebo_mi_experiment.launch
roslaunch experiments_launch mi_control.launch
````
# Running real world experiment
Docs and code soon
