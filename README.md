# Hierarchical Mixed-Initiative (MI) controller
The package "experiments_launch" contains all the top level .launch files. The package "variable autonomy" contains the structure of the hierarchical fuzzy controller.

# Setting up & getting started

1) Create a ROS workspace

2) Add navigation stack to 'src' directory :
```sh
$ git clone https://github.com/ros-planning/navigation.git
$ cd navigation
$ git checkout origin/melodic-devel
```

3) Add husky drivers to 'src' directory :
```sh
$ git clone https://github.com/uob-erl/husky.git
$ cd husky
$ git checkout origin/learning_effect_exp
```

4) Add package related to simulating robots in Gazebo to 'src' directory :
```sh
$ git clone https://github.com/uob-erl/erl_gazebo.git
```

5) Add fuzzy_MI_controller to 'src' directory :
```sh
$ git clone https://github.com/uob-erl/fuzzy_mi_controller.git
$ cd fuzzy_mi_controller
$ git checkout origin/hierarchical_mi
```

6) Add operator_intent_packages to 'src' directory :
```sh
$ git clone https://github.com/uob-erl/operator_intent_packages.git
$ cd operator_intent_packages
$ git checkout origin/hierarchical_MI
```

7) Add deepgaze_ros (check https://github.com/uob-erl/deepgaze_ros for extra info) to 'src' directory :
```sh
$ git clone https://github.com/uob-erl/deepgaze_ros.git
$ cd deepgaze_ros
$ git checkout origin/melodic-devel
```


