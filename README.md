# experiment3_mi
Repository for the Mixed-Inititive experiments.

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

# Running simulated experiment

1) Run MORSE simulation via the morse_arena.py in experiments_launch->world

2) run on ROS the morse_arena.launch file in launch directory

# Running real world experiment

1) run on ROS the robot_nav.launch file on the robot. This will run all the neccesery nodes to control the robot via HI, teleop, automy.

2) run on ROS the operator.launch file on the OCU computer.. This will run all the nodes to control the robot from the OCU (joystick needed).

3) run on ROS the mi_control.launch file on the robot. This will enable the MI control to run on the robot.
