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

The SLAM module plays a role in perceiving and analyzing environment information gathered by navigation sensors. In a GPS-deny environment, Unmanned vehicles depend on the SLAM algorithm to get accuracy localization information. In general, SLAM algorithms can be divided into lidar SLAM and visual SLAM. The visual SLAM algorithm is low-cost and lightweight, which benefits from camera sensors. However, visual SLAM suffers mostly from illumination variation and motion distortion. The lidar SLAM methods, by contrast, have a stable performance in challanging environment. This is one important reason why engineers prefer lidar SLAM whether in autonomous driving or service robots. 

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

Here is a summary table for four lidar SLAM methods in our package. As shown in this table, there are two pure lidar SLAM methods(A-LOAM and F-LOAM) and two lidar-inertial SLAM methods(LIO-SAM and Fast-LIO2). 

| SLAM method | Lidar | IMU  | Front-end odometry | Back-end optimization | Loop-closure | feature | Processor |
| ----------- | ----- | ---- | ------------------ | --------------------- | ------------ | ------- | --------- |
| A-LOAM      |  &#10004;    | 	&#10006; | Edge-planar | &#10006; | &#10006; | benchmark | Intel |
| F-LOAM | &#10004; | &#10006; | Edge-planar | &#10006; | &#10006; | Faster than loam | Intel |
| LIO-SAM | &#10004; | &#10004; | Edge-planar | Factor graph optimization | &#10004; | Can handle aggressive rotation | Intel |
| Fast-LIO2 | &#10004; | &#10004; | direct | iterated Kalman filter | &#10006; | Lightweight | Intel/ARM |

<div align="center">
Table 1: A comparision of different SLAM methods in this package 
</div>

## Setup Guide

### Dependency

### Compilation

## Quick Test

## Updates

## TODO

## Related works




