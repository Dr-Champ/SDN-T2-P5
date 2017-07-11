# MPC Project Writeup

## The Model

The model of the vehicle's dynamics is the one given in the lecture as follows.

* x0 = x1 + (v1 ∗ cos(psi1) ∗ dt)
* y0 = y1 + (v1 * sin(psi1) * dt)
* psi0 = psi1 + (Lf / v1 * delta1 * dt)
* v0 = v1 + (a1 * dt)
* cte0 = f1 - y1 + (v1 * sin(epsi1) * dt)
* epsi0 = psi1 - psides1 + (v1 * delta1 / Lf * dt)

Note that I decided to use indice "0" to represent the current timestep *t*, and indice "1" to represent the previous timestep *t-1*. These have been implemented in *MCP.cpp*, line 139-144.

The state vector comprises of 6 components as follows. 

* px - The car's x location in car's coordinate.
* py - The car's y location in car's coordinate.
* psi - The car's heading (rad) measured ccw from the car's x axis.
* v_mps - The car's speed in mps.
* cte - The car's Centerline Tracking Error, which is the difference in y axis between the location where the car is supposed to be and its current location, also in car's coordinate.

The state vector is initialized in the *main.cpp*, line 193. Note that transformation from global coordinate to car's coordinate system has been done earlier to all waypoint coordinates using a static helper function *transformGlobalToCarCoordinates()* at line 175. The px, py and psi given in the initial state are also set to 0 as the car is at the center of the car's coordinate system, headed toward the +x axis. 

The actuator signals are steering angle and acceleration (or break). These are returned from MPC.Solve() as the first 2 values in the returned vector, while the rest of the vector being the x and y coordinates of the prodicted trajectory of the car (in car's coordinate).

## Timestep Length and Elapsed Duration (N & dt)

The timestep length and duration are set in *MCP.cpp*, line 9-10 to 25 and 0.1 second respectively. These values were chosen after some trial-and-error runs as a balance between optimization / path prediction accuracy and my machine computing power to allow real-time optimization. After trying with several values of *N*, I've found that my machine can perform reasonably well in real-time with *N* no more than 30.  

This combination allows us to make optimization for 2.5-3 seconds into the future which seems to be a reasonable value given the desired speed of around 16 mph.


## Polynomial Fitting and MPC Preprocessing

Polynomial fitting to waypoints returned from the simulator is done at line 168-183 of *main.cpp* file. The waypoints are first transformed from global to car's coordinate system. Then a polynomial degree 4 is fitted to the waypoints. The reason a 4-degree polynomial was chosen is that it fits better to the curves at some parts of the road where a 3-degree polynomial doesn't.

Note that the waypoints are also adjusted to take into account the actuator latency when transformed into the car's coordinate and fitted to polynomial equation. See the next section for details. 

## Model Predictive Control with Latency

The model takes latency, say *T*, into account by predicting where the car would be, its orientation and speed in *T* seconds into the future at line 163-165 of *main.cpp*. Then, the reference waypoints are transformed from global to car's coordinate system using the newly adjusted car's position and orientation as the new origin point. 

The newly adjusted waypoints are then used for calculating polynomial coefficients to feed into the *MPC.Solve()*. The optimizer will then optimize the actuator signals to best control the car to follow the adjusted waypoints without us having to modify anything in the *FG_eval* and *MPC* classes. 