## Model Predictive Control of a 2 DOF planar robot arm

This ROS package implements a Model Predictive Controller on a two DOF planar robot arm. 
The setup can be used for implementing other controllers as well. 
Calculation of the dynamic model is based on kinetic and potential energies. Euler-Lagrange equations are used to compute the equations of motion. 
The final dynamic model depends on the arm's moment of inertia, coriolis force and gravitation force. [2]
The nonlinear dynamic model is multivariable (with two control inputs and two controlled outputs) so a two step design is proposed 
where we first develop a feedback linearisation control and then design the controller. [3]

#### Installation
Build package from source: navigate to the source folder of your catkin workspace and build this package using:
	
```
$ git clone https://github.com/28shambhavi/arm_mpc.git
$ cd ..
$ catkin_make
```

#### Running the Simulation

1. Arm model used can be changed by modifying the urdf in `urdf/2linkrobot.urdf`. It is defined as base>arm1>arm2>endEff.

2. Navigate into the launch folder. Launch the desired controller in the rviz environment. 
  ```
  $ roscd arm_mpc/launch
  $ roslaunch control_display.launch
  ```

3. Using the controllers: The launch file named `display.launch` launches the forward kinematics controller from `scripts/controller.py` for any point in the arm's workspace. The second launch file, `control_display.launch` will launch the MPC controller from `scripts/controller_mpc.py` and show the path from an initial point (both angles making zero degrees to the x axis) to the desired location of the end effector.

<img src="https://i.imgur.com/MuA3Npi.gif" width="600"/>

#### Built with
1. Python3
2. ROS Noetic

#### References: 
1. [Murray, Richard; A Mathematical Introduction to Robotic Manipulation](http://www.cds.caltech.edu/~murray/mlswiki/index.php/Main_Page "Reference")
2. David I, Robles G. PID Control Dynamics of A Robotics Arm Manipulator with Two Degrees of Freedom.Control De Procesos y Robótica. 2012, pp. 1–7.
3. Guechi E-H, Bouzoualegh S, Zennir Y, Blažič S. MPC Control and LQ Optimal Control of A Two-Link Robot Arm: A Comparative Study. Machines. 2018; 6(3):37. https://doi.org/10.3390/machines6030037

#### Authors
Shambhavi Singh ( [GitHub](https://github.com/28shambhavi), [Website](https://singh-shambhavi.github.io) )
