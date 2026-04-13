# Probabilistic Design Optimization Final Project: Trajectory Optimization Under Uncertainty for Autonomous Surface VEssel
> Ivy Mahncke

## Executive Summary

In this project, I use optimization to find the most efficient sequence of motion control commands for an Autonomous Surface Vessel (ASV) to execute as it explores designated waypoints over regions of scientific interest in a disturbance-heavy marine environment. I formulate the problem as a multi-objective optimization problem where distance from waypoints, elapsed time, and energy consumption are all costs to minimize. I also constrain the problem by designating off-limit areas such as coastlines or recreational marine areas, as well as maximum values for expended time and energy. I also model uncertainty that is present in the ASV's ability to sense its precise location, sense environmental disturbances, and execute desired motor commands.

## Background

### Problem Context

### Tenchi Diagram

<img src="./images/final_tenchi.JPG" width="600">

## Formulation

### Objective Function

The objective function describes the multi-objective minimization of A) physical position of the ASV in relation to its next waypoint, B) time elapsed at the point of mission completion, and C) energy consumed at the point of mission completion.

$$min f(u)=E[J_{position}+J_{time}+J_{energy}]$$

> where:

$$J_{position}=||p_k-p_{waypoint,s_k}||$$

$$J_{time}=T_{elapsed}=dt*k$$

$$J_{energy}=\sum_{k=0}^{k}\frac{1}{2}mv_k^2$$

The decision variable driving the optimization is the sequence of control vectors that dictate the ASV's motion in physical space. The control vector contains a linear velocity term and an angular velocity term.

> w.r.t.

$$u=[u_{0}:u_k]=[v_{0}:v_k,\omega_{0}:\omega_k]$$

The optimization is constrained by several factors, including off-limits regions of physical space, limitations on time and energy consumption, and upper bounds on control vector values.

> s.t.

$$p \notin P_{off-limits}$$

$$J_{time}\leq T_{max}$$

$$J_{energy}\leq E_{max}$$

$$|v_k|\leq v_{max}$$

$$|\omega_k|\leq \omega_{max}$$

### ASV Kinematics

The objective function is minimized by a sequence of control vectors, but the ASV's success is defined by its sequence of positions, which is a related but not identical value. Kinematic equations relate the ASV's position to control vectors as well as environmental disturbances. As such, all variables referenced here represent actual values, rather than values measured or intended by the ASV, unless stated otherwise.

#### State Spaces

State spaces describe sets of variables that collectively describe a key attribute about a model. The state spaces below act as a key to relate individual variables and the behaviors they influence together.

The ASV state describes key values about the ASV's position, orientation, and motion in physical space. It is comprised of the ASV's x position, y position, heading, linear velocity, and angular velocity.

> ASV state: \

$$x_k=[p_x, p_y, \theta, v, \omega]$$

The ASV control vector describes the ASV's intention of movement, which is imperfectly executed by motors in the physical world. It is comprised of the ASV's commanded linear velocity and commanded angular velocity.

> ASV control: \

$$u_k=[v_{k,cmd}, \omega_{k,cmd}]$$

The environmental disturbance vector describes the environment's physical influence on the ASV's motion, modeled as velocity. It is comprised of an x velocity component and a y velocity component.

> Environmental disturbance: \

$$d=[d_x,d_y]$$

#### Motion Kinematics

These are the set of equations governing the ASV's state transition after each timestep. These relate the ASV's next state to its current state, control vectors, and active disturbances.

$$p_{x,k+1}^{}=p_{x,k}+(v_kcos(\theta_k)+d_x)*dt$$

$$p_{y,k+1}=p_{y,k}+(v_ksin(\theta_k)+d_y)*dt$$

$$\theta_{k+1}=\theta_k+\omega_k*dt$$

$$v_{k+1}=v_{cmd}$$

$$\omega_{k+1}=\omega_{cmd}$$

### Waypoint Iteration

In the objective function, the ASV's distance to its next waypoint is a critical factor to minimize. As such, the model must iterate through a set of waypoints throughout the optimization process.

$$W=\{p_{waypoint,1},...,p_{waypoint,n}\}$$

The current waypoint is defined as a function of navigation state, which changes as waypoints are achieved by the ASV.

> current waypoint: \

$$p_{waypoint,s_k}$$

> where: \

$$s_{k+1}=\left{
  \begin{array}{lr}
    s_k+1: & |p_k-p_{waypoint,s_k}| < \epsilon \\
    s_k: & otherwise
  \end{array}
\right.$$


### Uncertainty

*TODO: Gaussians for now; dig into more accurate modeling later*

#### Destination Uncertainty
> Deployment point location uncertainty: \

$$p_{deploy}^{actual}=p_{deploy}^{desired}+\mathcal{N}(0,\sigma_{pos}^2)$$

>Waypoint location uncertainty: \

$$p_{waypoint,s_k}^{actual}=p_{waypoint,s_k}^{desired}+\mathcal{N}(0,\sigma_{pos}^2)$$

#### Measurement Uncertainty
> Motion control uncertainty: \

$$u_k^{actual}=u_k^{cmd}+\mathcal{N}(0,\sigma_{cmd}^2)$$

> Localization uncertainty: \

$$p_k^{measured}=p_k^{actual}+\mathcal{N}(0,\sigma_{GPS}^2)$$

> Disturbance uncertainty: \

$$d^{measured}=d^{actual}+\mathcal{N}(0,\sigma_{ADCP}^2)$$

### Constants

#### Physical Constants
Physical constants derived from the [BlueRobotics BlueBoat datasheet](https://bluerobotics.com/wp-content/uploads/2023/03/BLUEBOAT-DATASHEET-v1.1-JAN-2025.pdf) and the [BlueRobotics battery](https://bluerobotics.com/store/comm-control-power/powersupplies-batteries/battery-li-4s-18ah-r3/) assuming 2 batteries and no payload, with a maximum usage limit equal to half of the ASV's total capacity.

$M=16.82$ kg\
$T_{max}=9$ h\
$E_{max}=266$ Wh\
$v_{max}=3\frac{m}{s}$ \
$\omega_{max}=2\pi\frac{rad}{s}$

#### Uncertainty Constants

*TODO: Research motors and sensors datasheets for these values* \
$\sigma_{cmd}=???$ \
$\sigma_{GPS}=???$ \
$\sigma_{ADCP}=???$

#### Modeling Constants
Modeling constants are chosen for their utility in balancing a smooth and realistic model. \
$dt=0.1$ s

## Methodology

### Optimization Method

### Simulation Environment

## Results

## Conclusion

## Sources
### Accurate Modeling
[BlueRobotics BlueBoat datasheet](https://bluerobotics.com/wp-content/uploads/2023/03/BLUEBOAT-DATASHEET-v1.1-JAN-2025.pdf) \
[BlueRobotics battery](https://bluerobotics.com/store/comm-control-power/powersupplies-batteries/battery-li-4s-18ah-r3/) \
