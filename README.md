# Path Planer node
ROS node to interface with an ASV path planner

## Requirements
This package is meant to work in CCOMJHC's Project 11 simulation environment, the installation instructions for which are found <a href="https://github.com/CCOMJHC/project11_documentation/blob/master/SettingUpASimulationEnvironment.md">here</a>. It also uses my MPC ROS node, found <a href="https://github.com/afb2001/mpc">here</a>.

I've only ever used it on Ubuntu 18.04, so try anything else at your own risk.

## Installation
Install the <a href="https://github.com/CCOMJHC/project11_documentation/blob/master/SettingUpASimulationEnvironment.md">CCOMJHC Project 11 simulation environment</a>. MOOS-IvP components  are not required. Consider cloning my forks of the CCOMJHC/project11 and mission_manager repositories, as you'll need my changes from them to run my nodes.

Clone this repository and the MPC ROS node into the project11 catkin workspace:
```
cd ~/project11/catkin_ws/src
git clone https://github.com/afb2001/path_planner.git
git clone https://github.com/afb2001/mpc.git
```

You will also need the changes in my forked versions of the CCOMJHC project11 repo https://github.com/afb2001/project11.git, dubins_curves repo https://github.com/afb2001/dubins_curves.git, and mission_manager repo https://github.com/CCOMJHC/mission_manager.git. How you get them is up to you, but here's a helpful page about <a href="https://git-scm.com/book/en/v2/Git-Basics-Working-with-Remotes">working with remotes in git</a>.

I have a test scenario runner node that's in development, which is <a href="https://github.com/afb2001/test_scenario_runner.git">here</a>, and some test files for it <a href="https://github.com/afb2001/planner_test_suites.git">here</a>, although at the time of this writing they won't all work together.

If everything works out you should be able to run <code>catkin_make</code>. 

NOTE: I have a repository of undocumented semi-maintained scripts that can do most of the installation process for you <a href="https://github.com/afb2001/useful_scripts.git">here</a>. If you plan to run any of them, please look at them first. I created them for my own personal use, since I've needed to set up this system many times, but maybe you'll find them helpful too.

## Usage
Instead of using the launch file <code>project11/sim_local.launch</code>, you should use my new launch file: <code>project11/sim_path_planner_local.launch</code>. Now, when nav objectives are received in autonomous mode they will be sent to the path planner, and controls will be published to /helm by the MPC node. The hover action is not supported. 
