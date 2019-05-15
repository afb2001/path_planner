# Path Planer node
ROS node to interface with an ASV path planner

## Requirements
This package depends (for now) on <a href="https://github.com/afb2001/CCOM_planner">my Go planner</a> and is meant to work in CCOMJHC's Project 11 simulation environment, the installation instructions for which are found <a href="https://github.com/CCOMJHC/project11_documentation/blob/master/SettingUpASimulationEnvironment.md">here</a>. It also uses my MPC ROS node, found <a href="https://github.com/afb2001/mpc">here</a>.

I've only ever used it on Ubuntu 18.04, so try anything else at your own risk.

## Installation
Install both the <a href="https://github.com/CCOMJHC/project11_documentation/blob/master/SettingUpASimulationEnvironment.md">CCOMJHC Project 11 simulation environment</a> and <a href="https://github.com/afb2001/CCOM_planner">my Go planner</a>. MOOS-IvP components  are not required. Be sure to set your <code>GOPATH</code> to be <code>$HOME/go</code> and clone the Go planner there, or be prepared to edit the source files of this project in order to get it to work. Consider cloning my forks of the CCOMJHC/project11 and mission_manager repositories, as you'll need my changes from them to run my nodes.

Clone this repository and the MPC ROS node into the project11 catkin workspace:
```
cd ~/project11/catkin_ws/src
git clone https://github.com/afb2001/path_planner.git
git clone https://github.com/afb2001/mpc.git
```

You will also need the changes in my forked verisons of the CCOMJHC project11 repo https://github.com/afb2001/project11.git and mission_manager repo https://github.com/CCOMJHC/mission_manager.git. How you get them is up to you, but here's a helpful page about <a href="https://git-scm.com/book/en/v2/Git-Basics-Working-with-Remotes">working with remotes in git</a>.

If everything works out you should be able to run <code>catkin_make</code>. I also wrote a simplified build script for the Go planner which you can run by calling
```
~/go/src/github.com/afb2001/CCOM_planner/build_planner.sh
```

NOTE: if your version of the go planner is in a different place, that script will not work and neither will the path planner ROS node. If that's the case, you'll need to run some version of <code>go build</code> on your own and edit ~/project11/catkin_ws/src/path_planner/src/executive/executive.cpp line 211:
```
communication_With_Planner.set(string(homedir) + "/go/src/github.com/afb2001/CCOM_planner/planner", true, true, false, false);
```
to be
```
communication_With_Planner.set("path/to/your/version/of/the/planner/executable", true, true, false, false);
```

## Usage
Instead of using the launch file <code>project11/sim_local.launch</code>, you should use my new launch file: <code>project11/sim_path_planner_local.launch</code>. Now, when nav objectives are received in autonomous mode they will be sent to my path planner, and controls will be published to /helm by my MPC node. The hover action is not supported. 
