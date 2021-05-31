# NCRL_LIO
![Image](LOGO.png?raw=true "Title")
## Arthors
* Chun-Jung Lin - chadlin.gdr07g@nctu.edu.tw
* Jung-Cheng Yang - johnsongash.gdr07g@nctu.edu.tw
* Yin-Long Yan - yanlong658.gdr08g@nctu.edu.tw
* Bing-Yuan You - physical31031.c@nycu.edu.tw

## Preamble
NCRL_LIO is a package aimed to solve SLAM problem in real time with lidar-inertial system.

## Result
#### Indoor Experiment
[![Indoor Result](https://img.youtube.com/vi/X2338T96Tz8/0.jpg)](http://www.youtube.com/watch?v=X2338T96Tz8)

#### KITTI Dataset
[![KITTI Result](https://img.youtube.com/vi/B24UWVwRyC8/0.jpg)](http://www.youtube.com/watch?v=B24UWVwRyC8)

## Prerequisites
1. ROS Kinetic or Melodic [ROS INSTALLATION](http://wiki.ros.org/ROS/Installation)
2. Ceres Solver [CERES INSTALLATION](http://ceres-solver.org/installation.html)
3. PCL [PCL INSTALLATION](https://pointclouds.org/downloads/)

## Build ==NCRL_LIO==
1. `$ cd [workspace]/src`
2. `$ git clone https://chunjonglin@bitbucket.org/chunjonglin/ncrl_lio.git`
3. `$ catkin_make`
4. `$ source [workspace]/devel/setup.bash` or
`$ source [workspace]/devel/setup.zsh`

## Run ==NCRL_LIO==
#### operate with VLP-16
1. modify the parameters in `ncrl_lio/config/[filename.yaml]`
2. `roslaunch ncrl_lio run.launch`
#### operate with KITTI dataset
1. modify the parameters in `ncrl_lio/config/[filename.yaml]`
2. `roslaunch ncrl_lio KITTI.launch`

