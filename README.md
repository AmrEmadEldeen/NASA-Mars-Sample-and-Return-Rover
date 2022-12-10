# Introduction
In this project, we’ll do computer vision for robotics. We are going to build a Sample & Return Rover insimulation. Mainly, we’ll control the robot from images streamed from a camera mounted on the robot.
The project aims to do autonomous mapping and
navigation given an initial map of the environment.
Realistically speaking, the hard work is done now that
you have the mapping component! You will have the
option to choose whether to send orders like the
throttle, brake, and steering with each new image the
rover's camera produces.

The project will be submitted in two phases


### Phase 1 -Basic Operation-
The inset image at the bottom right when you're running in autonomous mode is packed with
information. In this image, your map of navigable terrain, obstacles and rock sample locations is
overplotted with the ground truth map. In addition, some overall statistics are presented
including total time, percent of the world you have mapped, the fidelity (accuracy) of your map,
and the number of rocks you have located (mapped) and how many you have collected.

The requirement for a passing submission of the first phase is to map at least 40% of the
environment at 60% fidelity and locate at least one of the rock samples (note: you're not required
to collect any rocks, just map the location of at least 1). Each time you launch the simulator in
autonomous mode there will be 6 rock samples scattered randomly about the environment and
your rover will start at random orientation in the middle of the map.



### Phase 2 -Collect & Return-
In this stage, build upon your previous implementation to map at least 95% of the environment at 85%
fidelity. All while colliding with the least number of obstacles. (The maximum number of collisions
allowed will be announced at the beginning of phase 2)

Also, there is a robotic arm located on the vehicle. In this phase, you should also locate and use the
robotics arm to pick up at least five rocks out of the six, and then return them back to the start position.

<br>

# Simulator
Here you can download the simulator on Linux: <a href="https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip" target="_blank">The simulator</a>.

<br>

# Team Members
- Yousef Amer Awadallah Osman (ID: 1901524)
- Ahmed Yasser Hassan  (ID: 1809986)
- Amr Emad ElDeen Abdel Ghafaar (ID: 1803188)
- Mohamed Farrag Mohamed (ID: 16P8192)
- Abdelrahman Osama Mohamed (ID: 1806900)
