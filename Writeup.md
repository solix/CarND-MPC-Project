# Report 
MPC-PROJECT 


### Model 
For MPC project we are using an optimizer to find the control inputs for steering angle and acceleration of the car that drives in the simulation.
We only execute first set of control inputs this brings the vehicle to the new state and then it repeats a process by optimizing the cost function.
#### State:

Process starts by initiating the state of the vehicle by reading data from simulator, for MPC we will need six states to get a correct prediction and robust driving. these variables will be feed into the Solver (optimizer) that returns newly calculated state. These variables are  vehicle position `x` and `y` in car coordinate system, orientation `psi` , speed of the vehicle `v`, cross track error `cte` and orientation error `epsi`.

#### Actuators:

With introduction of state variables, we want to observe how model evolves through time and how we can modify control inputs `steer_value` and `throttle` value such that our cost function is as low as 0. Actuator control inputs helps us to control vehicle state. 

#### Preprocess: 
From the simulator besides getting the initial state of the vehicle data, way-points to follow as reference trajectory is given. With this data we can calculate errors for cross track and orientation of the car relative to the way-points. In order to make our life easier for calculating those we will shift the coordinates of the way-points and transform it to vehicle coordinate system. That makes `x` and `y` to be zero at our initial state. 

We then fit transformed way-points to `polyfit` function. this will give us a coefficient which help us calculate `cte` and `epsi`. 

To calculate initial `cte` There is a handy `polyeval(coefficient,0)` function (note that   `x` is 0). Basically this function give us a distance of the current `waypoints_X` to of car's `x` position. 

OK now in this moment all the variables are preprocessed and ready to feed into the optimizer to start reducing the cost function.

#### Set up Cost 

The following errors are influencing the cost function: Cross track error , heading and velocity. We additionally want to penalize our turns if it is two sharp and also we penalize high rate of change between sequential actuators. To make a model reach an optimal solution we need to tune weight for each error. Ultimate goal is to reduce cross track error and stay in track while keeping the velocity on certain rate because we dont want the car to stop in the middle of the way. I have tried various weight ratio. for example I penalized in the beginning the `cte` and `epsi` error, and left the rest light weighted. result was sharp turns and therefore car couldn't finally stay in track.  Also other try out was to penalize the speed `v` with higher rate while `cte` had also high wight. this was contradicting the control input and ended with slow turns on sharp edges.

This configuration gave a best result in the end of fine tuning cost variables:

| Error (type)      | Weight           | reference error  |
| ------------- |:-------------:| -----:|
| `cte_start`      |4000 |0 |
| `epsi_start`      | 5000      |   0 |
| `v_start`           |  0 | 100
| `delta_start` | 5|-|
| `a_start` | 5|-|
| `delta_start ` | 200| sequential (t+1) -t|
| `a_start` | 200|sequential (t+1) -t|


#### N & dt:
choosing T = N*dt is also an important aspect in of model for real-time calculations,  if `N` is big and `dt` is small then optimizer requires more time to calculate the next optimal solution. however setting a higher `dt` will result in wrong prediction since the time interval between two sequence is larger. Optimally system works pretty well with `N=10` and interval of `dt=0.1`.

#### Update:
For the update step Ipopt (Interior Point OPTimizer, pronounced eye-pea-Opt) is being used. This open-source project is designed to find (local) solutions of mathematical optimization problems. To setup things for Ipopt in order to make it start working, we need to feed cost, all the variables, actuator and set of constraints.

There are set of equations for new state at time `t+1` these need to be calculated and feed in to the optimizer in order to have a reference that eventually returns optimal solution. We enforce state vector variables to be equal to the previous time steps plus the rate of change. for example to enforce `x_t+1` to be essentially equal to previous x multiplied by rate of change, we have `x_t+1 = x_t + v_t * cos(psi) * dt `;

#### Solver & Constraints
We feed the state vector along with coefficients that we calculated from way-points to the optimizer. Optimizer requires set of threshold to begin with. We define upper-bound and lower-bound for variables and constraints. These values is just required by solver and we initialize high and low value for all of our state variables, for actuator we take into account ranges `-/+30 degree` for steering angle and `-/+1.0` for throttle. We then add all the lower and upper bound constraints to the values received from initial state.

#### Dealing with latency:
There is `100ms` delay for the actuations control to take effect. The solution that I came up with was to calculate new set of state variables before feeding them into the state vector. I used same update equation as for next time state `t+1` and feed the new variables as such:

```
// predicts state in 100ms. note we start x, y with 0 in our state
          double latency = 0.1;
          double px_f = 0.0 + v * latency;
          double py_f = 0.0;
          double psi_f = 0.0 + v *  -steer_value / Lf * latency; //future psi
          double v_f = v + throttle_value * latency;
          double cte_f = cte + v * sin(epsi) * latency;
          double epsi_f = epsi + v * -steer_value / Lf * latency;



Like this we actuate at the right time taking delay in to account.
