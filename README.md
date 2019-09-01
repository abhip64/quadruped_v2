# Quadruped v2.0

This projects aims to develop a simulation environment capable of testing out different static gait strategies of quadruped with insect-like leg configuration.  The project has been carried out using open source platforms - **ROS** and **Gazebo**.

The project runs in concurrence with an effort to develop a hardware prototype of a legged wall climbing robot. The purpose of the simulation in context with the hardware prototype is two-fold. Firstly, it should serve as an effective and efficient way to test out different leg configurations and gait strategies aiding in its design. Secondly, it should also serve as a **digital twin** for the finally design hardware of the legged walker.
For those who are interested more details on digital twins can be obtained from the link given below:

This github wiki associated with this repo is meant as a detailed guide for anyone who wants to dive deep into quadrupedal locomotion strategies using statically stable gaits and also on setting up **ROS** and **GAZEBO** for developing their own simulations . Detailed documentation of the code will also be provided so that anyone can clone this repo and modify the code for their own use.

## Software Pre-Requisites
The project has been developed  on an Intel i5 dual core laptop running Ubuntu 16.04. The following versions of software were used - 

 - ROS Kinetic
 - Gazebo 9.5

> Kindly note that the code has not been tested and validated for any other versions of the above mentioned software.  Different Gazebo versions may not create problems but a different ROS distribution might. 

## Current Status of the Project
**The project is under active development**

The simulated quadruped is able to follow a specified 2D path using a crab-turning gait.  The code also has been successfully used to control a quadruped prototype that has been developed in our lab. There are also problems relating to following curved paths in the simulation. It has to be investigated further.

## Future Improvements
 For achieving the complex task of wall climbing the quadruped must be equipped with the capability to traverse a 3D environment. Thus it is necessary to develop 3D gaits enabling the legged robot to climb on a general 3D surface and also carry out plane change manoeuvres. There are also plans to  extend the system to hexapods which offer a more stable and flexible platform compared to quadruped but with the added complexity of controlling 6 legs instead of 4.
 
 Update in the codes required :
 
 - Remove the need for param.yaml to update the link lengths in parameter server. Rather use **OROCOS** library to update these values using urdf.

<!--stackedit_data:
eyJoaXN0b3J5IjpbLTM1MDcxNDYxMCwtMjk0MjA1NzcsLTE5Mz
M4Njg5NzIsMTMzNjY4MDM3NiwyMDI3MTE2MzUxXX0=
-->
