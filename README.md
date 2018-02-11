# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Description

This repository contains my implementation of the Model Prective Control (MPC) project of the Self-Driving Car Engineer Nanodegree Program in C++.

The program takes input from a the Udacity car simulator (2D car position, heading, velocity, cross-track error and heading error) and computes and feeds the control values (steering angle and acceleration) back.

## Steps
1. As the input waypoints which mark the middle of the road are in world coordinates we first convert them to the car's coordinate system. This greatly simplifies the calculations.
2. We have to fit an at least order 3 polynomial on the waypoints (actually, as stated in http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf, an even higher degree (quintic) might be necessary to get optimal results)
```c++
auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
```
3. Next, we have to calculate the cross-track error and the heading error using the polynomial
```c++
double cte = polyeval(coeffs, 0);  // px = 0, py = 0
double epsi = -atan(coeffs[1]);  // p 
```
---
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

