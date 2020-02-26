# Path Planer node
ROS node to interface with an ASV path planner

## Requirements
This package is meant to work in CCOMJHC's Project 11 simulation environment, the installation instructions for which are found <a href="https://github.com/CCOMJHC/project11_documentation/blob/master/SettingUpASimulationEnvironment.md">here</a>. It also uses my MPC ROS node, found <a href="https://github.com/afb2001/mpc">here</a>.

I've only ever used it on Ubuntu 18.04, so try anything else at your own risk.

## Installation
Install the <a href="https://github.com/CCOMJHC/project11_documentation/blob/master/SettingUpASimulationEnvironment.md">CCOMJHC Project 11 simulation environment</a>. MOOS-IvP components  are not required.

Clone this repository and the MPC ROS node into the project11 catkin workspace:
```
cd ~/project11/catkin_ws/src
git clone https://github.com/afb2001/path_planner.git
git clone https://github.com/afb2001/mpc.git
```

I have a test scenario runner node that's in development, which is <a href="https://github.com/afb2001/test_scenario_runner.git">here</a>, and some test files for it <a href="https://github.com/afb2001/planner_test_suites.git">here</a>.

If everything works out you should be able to run <code>catkin_make</code>. 

NOTE: I have a repository of undocumented semi-maintained scripts that can do most of the installation process for you <a href="https://github.com/afb2001/useful_scripts.git">here</a>. If you plan to run any of them, please look at them first. I created them for my own personal use, since I've needed to set up this system many times, but maybe you'll find them helpful too.

## Usage
See the simulation environment documentation for instructions on getting that running. My primary nodes are included in the standard launch files. Once everything is running, use the dynamic_reconfigure window to switch the path follower to my planner.
