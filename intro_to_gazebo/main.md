% UR5e in Gazebo and MoveIt!

- Author: Tejaswi Digumarti (tejaswi.digumarti@sydney.edu.au)
- Last Updated: 17th May 2020

## Foreword
This document provides explains how to simulate a UR5e arm in Gazebo and control it using MoveIt via ROS.
I have tried to provide a high-level overview of the fundamentals of simulation in Gazebo and go into implementation details only for the UR5e arm.
Similar is the case for MoveIt!
To keep this document concise, I have provided links to resources wherever I felt that the content was already presented in a good form and a repetition of it in this document is not necessary.
I encourage the reader to check them out for a complete understanding of the topic.

Instead of starting from scratch and building our way up, I will present the content in a top-down approach where we will begin with a fully functioning simulation and then disect it to understand the various parts of the framework, and how they interact with each other.

The target audience is the students of the MTRX5700 - Experimental Robotics course at the University of Sydney.
This document describes how the simulation framework for Assignments 1 and 2 was build and how you can modify it for your major projects.
While only UR5e is covered in detail, the document is also useful for those who are simulating other robots.

## Introduction
### Gazebo
Gazebo was originally developed for the purpose of simulating outdoor environments with high-fidelity, by Dr. Andrew howard and Nate Koenig at the University of Southern California in 2002 [[ref]](http://gazebosim.org/). It was part of the [Player/Stage Project](http://robotics.stanford.edu/~gerkey/research/final_papers/icar03-player.pdf) along with Player (clean interface to robots and sensors) and Stage (multiple robots simulator). In the subsequent years it gained independent popularity and has grown to become the go to simulator in Robotics for both indoor and outdoor environments. As a result several robots and environmets have been simulated in Gazebo and are available for use by the community. 

Some popular ones are   
- [PR2](http://wiki.ros.org/pr2_simulator/Tutorials)  
- [Pioneer P3DX](https://github.com/RafBerkvens/ua_ros_p3dx)  
- [Baxter](https://github.com/RethinkRobotics/sdk-docs/wiki/Using-Gazebo-and-Baxter)  
- [TurtleBot](http://wiki.ros.org/turtlebot_gazebo/Tutorials/indigo/Explore%20the%20Gazebo%20world)  
- [UR5](https://github.com/ros-industrial/universal_robot)  
- [RotorS](https://github.com/ethz-asl/rotors_simulator/wiki)  
- [Duckietown](https://github.com/duckietown/duckietown-sim-server)  

In recent years, there are also a growing number of simulators that are taking advantage of game engines like Unity and Unreal Engine, especially for their high-fidelity graphics capabilities (I personally use Unreal Engine). These are typically used where photo-realism is desired. In cases where photo-realism is an overkill, Gazebo is still the simulation environment of choice for modelling many robots and enviornments.

### Gazebo and ROS
Gazebo is a standalone simulation framework that can be used to model objects, mechanisms, environments and the interaction between them, i.e. physics. Hence it can be used to model robots. It is not necessary to use Gazebo with ROS, but using it along with ROS makes it interesting for a few reasons. In the context of this course, the most important reason is that Gazebo can be used as a drop in replacement for the real world, provided that the modelling is done reasonably well. One can use the same code written using ROS to test in simulation and directly deploy it on the real robot.

### MoveIt!
MoveIt! is a motion planning and manipulation toolkit for ROS [[ref]](https://moveit.ros.org/), initially developed at Willow garage in 2011. It is now an open source project with several companies and universities contributing and maintianing it.

In the context of this document, we will be using MoveIT! to control the UR5e arm.

## Building Simulations
This is probably what you are reading this document for.
I will cover this section in detail taking the example of the simulation environment for Assignments 1 and 2 that uses the UR5e arm.
For other robots or environments, please refer to one of the links in the introduction section.
*Note: Some of the frameworks may be outdated and no longer be maintained, but are still useful from an educational point of view.*

The main references for this section are the [Gazebo Tutorials(http://gazebosim.org/tutorials?cat=guided_b) and [MoveIt! Tutorials](https://ros-planning.github.io/moveit_tutorials/).
You can also continue reading this document and refer to them when in doubt.
I will cover some fundamentals in this document as well.

Let us now take a dive directly into the simulation framework and try to understand how everything works.

### Filesystem overview
If you look at the code that was distributed with assignment 1 (available on canvas) in the folder **assignment_1_code**, it is organized as follows.

<center>
<img src="images/ass1_main.png" align="middle">
</center>
Figure 1 - Main folders and their contents in the code for assignment_1.

The two main folders are  
* **universal_robot**:  containing code relevant to the UR5e robotic arm such as its description in simulation, its configuration for MoveIt!, its forward kinematics, its CAD model etc.
* **assignment_1**: containing code specific to the assignment

#### universal_robot
We shall focus on three subfolders which are relevant to this dicussion.  
1. ur5_e_moveit_config: This is where the configuration files required by MoveIt are present
2. **ur_e_description**: This is where the files that describe the model of the arm are present  
3. **ur_e_gazebo**: This is where the launch files related to gazebo are present  

<center>
<img src="images/ur_root.png" align="middle">
</center>
Figure 1 - Folders in universal_robot

#### assignment_1
This fodler is organized as follows.

<center>
<img src="images/ass1_ass1_root.png" align="middle">
</center>
Figure 1 - Folders in assignment_1

### Workflow
When you run your demo or assignment script, the set of files that are used is shown in the picture below.

### How to modify the simulation


#### URDF
#### SDF
#### Textures
#### Models


