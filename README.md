# open-source-SLAM-package
A open-source SLAM package FOR CUHK UAS fundamental group

## Introduction

![image](https://user-images.githubusercontent.com/58619142/181909310-37a21609-c84e-4939-9f0b-bd07619eb319.png)


This package presents a set of state-of-art open-sourced lidar-based Simultaneous  Localization and Mapping(SLAM) software for UAV localization in a 3D factory environment, targeting for:
* Lightweight(both hardware and software);
* Robust;
* High precision;
* integration with other algorithms(navigation, task planning and so on).

Up to now, we have collected and tested with four kinds of lidar SLAM, including pure lidar SLAM algorithms and lidar-inertial SLAM algorithms. Integration of autonomous UAV software and development of low-cost SLAM algorithm will be our future work directions.

## Overall Architecture

<div align="center">
<img src="https://user-images.githubusercontent.com/58619142/181913836-e9d73f88-0804-47b0-bddc-5d68426b37e8.png"
     width="70%"
     alt="Autonomous UAV system overview"/>
<br>
Autonomous UAV system overview
</div>

<br/>

The SLAM module plays a role in perceiving and analyzing environment information gathered by navigation sensors. In a GPS-deny environment, Unmanned vehicles depend on the SLAM algorithm to get accuracy localization information. In general, SLAM algorithms can be divided into lidar SLAM and visual SLAM. The visual SLAM algorithm is low-cost and lightweight, which benefits from camera sensors. However, visual SLAM suffers mostly from illumination variation and motion distortion. This is one important reason why engineers prefer lidar SLAM whether in autonomous driving or service robots. 

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
Lidar-inertial SLAM system pipeline
</div>

<br/>

## Summary Table

## Setup Guide

### Dependency

### Compilation

## Quick Test

## Updates

## TODO

## Related works




