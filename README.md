# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Description

This repository contains my implementation of the Model Prective Control (MPC) project of the Self-Driving Car Engineer Nanodegree Program in C++.

The program takes input from a the Udacity car simulator (2D car position, heading, velocity, cross-track error and heading error) and computes and feeds the control values (steering angle and acceleration) back.

## Steps
1. As the input waypoints which mark the middle of the road are in world coordinates we first convert them to the car's coordinate system. This greatly simplifies the calculations.
2. Fit an at least order 3 polynomial on the waypoints (actually, as stated in http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf, an even higher degree (quintic) might be necessary to get optimal results)
```c++
auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
```
3. Calculate the cross-track error and the heading error using the polynomial. The cross-track error is the value of the polynomial at 0, and the error of the orientation is the arctan of the 2nd order coefficient. (The `epsi` value has to be negated because the simulator flips sides)
```c++
double cte = polyeval(coeffs, 0);  
double epsi = -atan(coeffs[1]);  
```
4. The state vector is fed to the MPC Solver function which calculates the control parameters using a model-based cost function and with respect to the control variable constraints. The most important factor here is the cost function because it is responsible for the jerk-free and smooth control of the car's motion through the weighted sum of the following terms:
```c++
    // The part of the cost based on the reference state.
    for (unsigned int t = 0; t < N; t++) {
      fg[0] += c_cte    * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += c_epsi   * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += c_v      * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    // Minimize the use of actuators.
    for (unsigned int t = 0; t < N - 1; t++) {
      fg[0] += c_delta  * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += c_a      * CppAD::pow(vars[a_start + t], 2);
      fg[0] += c_vd     * CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2);
    }
    // Minimize the value gap between sequential actuations.
    for (unsigned int t = 0; t < N - 2; t++) {
      fg[0] += c_ddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += c_da     * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```
The parameters ´c_cte´, ´c_epsi´, ´c_v´, ´c_delta´, ´c_a´, ´c_vd´, ´c_ddelta´ and ´c_da´ are the relative weights to scale each error term appropriately in the final cost sum. Terms with higher weights will have major, terms with lower weights will have minor influence on the car's behaviour.

5. The output of the solver contains the control parameters (steering angle and throttle) and the calculated waypoints (for displaying them in the simulator - the reference trajectory is yellow, the calculated is green). 



---
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

