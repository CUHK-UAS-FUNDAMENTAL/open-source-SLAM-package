# open-source-SLAM-package
An open-source SLAM integration package for [*CUHK UAS fundamental group*](http://www.mae.cuhk.edu.hk/~usr/)

## Menu


  * [1. Introduction](#1-introduction)
  * [2. Overall Architecture](#2-overall-architecture)
  * [3. Summary Table](#3-summary-table)
  * [4. Setup Guide](#4-setup-guide)
    + [4.1 Dependency](#41-dependency)
      - [4.1.1 General Dependency](#411-general-dependency)
      - [4.1.2 A-LOAM & F-LOAM](#412-a-loam---f-loam)
      - [4.1.3 LIO-SAM](#413-lio-sam)
      - [4.1.4 Fast-LIO2](#414-fast-lio2)
    + [4.2 Compilation](#42-compilation)
      - [4.2.1 download A-LOAM and build it](#421-download-a-loam-and-build-it)
      - [4.2.2 download F-LOAM and build it](#422-download-f-loam-and-build-it)
      - [4.2.3 download LIO-SAM and build it](#423-download-lio-sam-and-build-it)
      - [4.2.4 download Fast-LIO and build it](#424-download-fast-lio-and-build-it)
  * [5. Quick Test](#5-quick-test)
    + [5.1 Test with PX4 Fundamental  Simulator](#51-test-with-px4-fundamental--simulator)
  * [6. Updates](#6-updates)
  * [7. TODO](#7-todo)
  * [8. Related works](#8-related-works)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>



## 1. Introduction

![image](https://user-images.githubusercontent.com/58619142/182330364-2e35f0b0-6fae-4314-a9b9-2444ce8b64a7.png)


This package presents a set of state-of-art open-sourced lidar-based Simultaneous  Localization and Mapping(SLAM) software for UAV localization in a 3D factory environment, targeting:
* Lightweight(both hardware and software);
* Robust;
* High precision;
* integration with other algorithms(navigation, task planning, and so on).

Up to now, we have collected and tested four kinds of lidar SLAM, including pure lidar SLAM algorithms and lidar-inertial SLAM algorithms. Integration of autonomous UAV software and the development of a low-cost SLAM algorithm will be our future work directions.

## 2. Overall Architecture

<div align="center">
<img src="https://user-images.githubusercontent.com/58619142/182335315-06c7d47d-3a23-47c4-83db-734fab28334e.png"
     width="100%"
     alt="Autonomous UAV system overview"/>
<br>
Figure 1: Autonomous UAV system overview
</div>


<br/>

The SLAM module plays a role in perceiving and analyzing environment information gathered by navigation sensors. In a GPS-deny environment, Unmanned vehicles depend on the SLAM algorithm to get accurate localization information. In general, SLAM algorithms can be divided into lidar SLAM and visual SLAM. The visual SLAM algorithm is low-cost and lightweight, which benefits from camera sensors. However, visual SLAM suffers mostly from illumination variation and motion distortion. The lidar SLAM methods, by contrast, have a stable performance in a challenging environment. This is one important reason why engineers prefer lidar SLAM whether in autonomous driving or service robots. 

<br/>

In this project, we take more concern on the lidar or lidar-inertial fusion SLAM algorithm. The lidar SLAM mostly contains below sub-modules:
* offline calibration
* Point cloud preprocessing & sensor data preprocessing
* motion distortion compensation
* Front-end lidar odometry or lidar-inertial odometry
* Back-end optimization 
* Visualization

<div align="center">
<img src="https://user-images.githubusercontent.com/58619142/181915418-cd55e434-fdb4-4f30-98fb-386c2b324990.PNG"
     width="70%"
     alt="Lidar-inertial SLAM system pipeline"/>
<br>
Figure 2: Lidar-inertial SLAM system pipeline

<br/>
</div>

## 3. Summary Table

Here is a summary table for four lidar SLAM methods in our package. As shown in this table, there are two pure lidar SLAM methods(A-LOAM and F-LOAM) and two lidar-inertial SLAM methods(LIO-SAM and Fast-LIO2). 

| SLAM method | Lidar | IMU  | Front-end odometry | Back-end optimization | Loop-closure | feature | Processor |
| ----------- | ----- | ---- | ------------------ | --------------------- | ------------ | ------- | --------- |
| A-LOAM      |  &#10004;    | 	&#10006; | Edge-planar | &#10006; | &#10006; | benchmark | Intel |
| F-LOAM | &#10004; | &#10006; | Edge-planar | &#10006; | &#10006; | Faster than A-LOAM | Intel |
| LIO-SAM | &#10004; | &#10004; | Edge-planar | Factor graph optimization | &#10004; | Can handle aggressive rotation | Intel |
| Fast-LIO2 | &#10004; | &#10004; | Direct | Iterated kalman filter | &#10006; | Lightweight | Intel/ARM |

<div align="center">
Table 1: A comparison of different SLAM methods in this package 
</div>

## 4. Setup Guide

Before starting the test, we strongly recommend that users follow the following steps to configure the computer environment. These steps have been tested under **Ubuntu 64-bit 18.04**.

### 4.1 Dependency

#### 4.1.1 General Dependency

1. ROS
ROS Melodic:  [ROS Installation](http://wiki.ros.org/ROS/Installation)
2. Eigen
Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

#### 4.1.2 A-LOAM & F-LOAM

1. Ceres Solver
Follow [Ceres Installation](http://ceres-solver.org/installation.html).
2. PCL
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).
3. Trajectory visualization
For a visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed.

#### 4.1.3 LIO-SAM

1. ROS dependency

Open a new terminal and type the following command：
  ```bash
  sudo apt-get install -y ros-melodic-navigation
  sudo apt-get install -y ros-melodic-robot-localization
  sudo apt-get install -y ros-melodic-robot-state-publisher
  ```
  
 2. [GTSAM](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)

Again, type the following command:
  ```bash
  sudo add-apt-repository ppa:borglab/gtsam-release-4.0
  sudo apt install libgtsam-dev libgtsam-unstable-dev
  ```
  
  #### 4.1.4 Fast-LIO2
  
  1. livox_ros_driver
  Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).
  
  *Remarks:*
- Since the FAST-LIO must support Livox serials LiDAR firstly, so the **livox_ros_driver** must be installed and **sourced** before running any FAST-LIO launch file.
- How to source? The easiest way is add the line ``` source $Licox_ros_driver_dir$/devel/setup.bash ``` to the end of file ``` ~/.bashrc ```, where ``` $Licox_ros_driver_dir$ ``` is the directory of the livox ros driver workspace (should be the ``` ws_livox ``` directory if you completely followed the livox official document).

  
### 4.2 Compilation

Firstly, you need to create a ros workspace and initial it. 
Open a new terminal and type the following command：
  ```bash
  mkdir 3d_SLAM_ws && cd 3d_SLAM_ws
  mkdir src && cd src
  catkin_init_workspace
  cd .. && catkin_make
  ```

#### 4.2.1 download A-LOAM and build it 
Clone the repository and catkin_make:
```bash
    cd ~/3d_SLAM_ws/src
    git clone https://github.com/CUHK-UAS-FUNDAMENTAL/A-LOAM.git -b dev_v1.0
    cd ../
    catkin_make
```

#### 4.2.2 download F-LOAM and build it 
Clone the repository and catkin_make:
```bash
    cd ~/3d_SLAM_ws/src
    git clone -b gzx_dev https://github.com/CUHK-UAS-FUNDAMENTAL/F-loam.git 
    cd ..
    catkin_make
```

#### 4.2.3 download LIO-SAM and build it 
Clone the repository and catkin_make:
```bash
 cd ~/3d_SLAM_ws/src
 git clone -b gzx_dev https://github.com/CUHK-UAS-FUNDAMENTAL/LIO_SAM.git
 cd ..
 catkin_make
```

#### 4.2.4 download Fast-LIO and build it 
Clone the repository and catkin_make:
```bash
   cd ~/3d_SLAM_ws/src
    git clone https://github.com/hku-mars/FAST_LIO.git
    cd FAST_LIO
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver before build (follow 1.3 **livox_ros_driver**)
- If you want to use a custom build of PCL, add the following line to ~/.bashrc
```export PCL_ROOT={CUSTOM_PCL_PATH}```

## 5. Quick Test

### 5.1 Test with PX4 Fundamental  Simulator

The PX4 Fundamental  Simulator is a UAV simulator developed by [yizhou Chen 陈奕州](https://github.com/JINXER000). If you are not familiar with this simulator, I recommend that you get to start with this [wiki](https://github.com/JINXER000/FundamentalSimulatorPX4/wiki). Now,  the simulator is still under revision. More details can be seen [here](https://github.com/CUHK-UAS-FUNDAMENTAL/FundamentalSimulatorPX4/tree/xtdrone/gzx_dev).

![simulator environment](https://user-images.githubusercontent.com/58619142/182297052-886127c5-dbb1-455f-8755-402507335062.png)

<div align="center">
Figure 2:  a factory environment in PX4 fundamental simulator
</div>

Users can remotely control a small drone to explore a simulation environment(e.g., a factory). Meanwhile, the pose of UAV and a point cloud map incrementally constructed will be output by SLAM algorithms.  The program runs in the following order:

1. start the PX4 simulator
```bash
    roslaunch px4 powerplant_lidar.launch
```

2. run the control python file

```bash
cd ~/${usr_path_to_XTDrone}/XTDrone/control/keyboard/
python multirotor_keyboard_control.py iris 1 vel
```

3. run the communication python file
```bash
cd ~/${usr_path_to_XTDrone}/XTDrone/communication/
python multirotor_communication.py iris 0
```

4. run lidar SLAM algorithm

 * A-LOAM

```bash
  cd ~/3d_SLAM_ws
 source ~/3d_SLAM_ws/devel/setup.bash
 roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
```

 * F-LOAM

```bash
  cd ~/3d_SLAM_ws
 source ~/3d_SLAM_ws/devel/setup.bash
 roslaunch floam floam_gazebo.launch
```

* LIO-SAM

```bash
  cd ~/3d_SLAM_ws
 source ~/3d_SLAM_ws/devel/setup.bash
roslaunch lio_sam run.launch
```

* Fast-LIO

```bash
    cd ~/3d_SLAM_ws
    source ~/3d_SLAM_ws/devel/setup.bash
    roslaunch fast_lio mapping_velodyne.launch
```

## 6. Updates

2022-08-02 update the readme file.


## 7. TODO


## 8. Related works
* A-LOAM

[A-LOAM: Advanced implementation of LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)

 * F-LOAM

[F-LOAM: Fast LiDAR Odometry and Mapping](https://github.com/wh200720041/floam)
 
* LIO-SAM

[LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping](https://github.com/TixiaoShan/LIO-SAM)

* Fast-LIO

[ikd-Tree: An Incremental K-D Tree for robotic applications](https://arxiv.org/abs/2102.10808)

[Robust Real-time LiDAR-inertial Initialization](https://github.com/hku-mars/LiDAR_IMU_Init)

[IKFoM: Iterated Kalman Filters on Manifolds](https://github.com/hku-mars/IKFoM)

[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9697912)

[FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)





