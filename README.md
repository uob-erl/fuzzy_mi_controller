# experiment3_mi
Repository for the Mixed-Inititive experiments.

# Installing fuzzylite cpp library 

1) Download the v4.0 library

2) unzip the source code in order to build nd install library

3) cd into the fuzzylite folder

4) change the code in CMakeLists.txt file into:

```sh
install(TARGETS fl-bin fl-shared fl-static
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		)
```

5) run the following commands

```sh
$ mkdir -p release
$ cmake -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DFL_BACKTRACE=ON -DFL_USE_FLOAT=OFF -DFL_CPP11=ON
$ make
$ sudo make install
```

6) the library should work

# Running simulated experiment

1) Run MORSE simulation via the morse_arena.py in experiments_launch->world

2) run on ROS the morse_arena.launch file in launch directory

# Running real world experiment

1) run on ROS the robot_nav.launch file on the robot. This will run all the neccesery nodes to control the robot via HI, teleop, automy.

2) run on ROS the operator.launch file on the OCU computer.. This will run all the nodes to control the robot from the OCU (joystick needed).

3) run on ROS the mi_control.launch file on the robot. This will enable the MI control to run on the robot.