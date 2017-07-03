# MPC Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This project implements a MPC (Model Predictive Controller) to use with the Udacity car simulator.  With the MPC running, the car will autonomously drive around the track at high speeds.  The simulator includes visual indicators showing a yellow line of upcoming waypoints and a green line of path prediction based on the MPC solver.

![](mpc-run.gif)
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

# Project Goals and [Rubric](https://review.udacity.com/#!/rubrics/896/view)

The goals of this project are the following:

* The MPC procedure must be implemented as was taught in the lessons.
* A polynomial is fitted to waypoints
* Waypoints, vehicle state and actuators are preprocessed prior to the MPC procedure
* The system must handle a 100ms artificial latency
* The vehicle must successfully drive a lap around the track

# Implementation of the MPC

The lectures and quizzes for MPC covered a basic implementation of it.  These quizzes were condensed into a single [lab project](https://github.com/justinlee007/CarND-MPC-Lab) that realized most of the actual project.
 
MPC is a process that uses a realistic model of the vehicle dynamics to create a predicted path that follows a reference trajectory.  The predicted path is optimized based on a cost function that takes into account CTE and other constraints.

![](mpc-scheme-basic.png)
##### Basic MPC scheme (image from Wikipedia)

This MPC implementation consists of two components: **setup** and **loop**

## Setup

* Define the length of the trajectory, N, and duration of each timestep, dt.
* Define vehicle dynamics and actuator limitations along with other constraints.
* Define the cost function.

### Length of trajectory and timestep duration

This model uses **N=10** trajectory iterations at **100ms** for each timestep duration, effectively forecasting 1 second of path prediction.

### Constraints

* *x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> ∗ cos( ψ<sub>t</sub> ) ∗ dt*
* *y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> ∗ sin( ψ<sub>t</sub> ) ∗ dt*
* *ψ<sub>t+1</sub> = ψ<sub>t</sub> + v<sub>t</sub> / L<sub>f</sub> ∗ δ<sub>t</sub> ∗ dt*
* *v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> ∗ dt*
* *cte<sub>t+1</sub> = f(x<sub>t</sub>) − y<sub>t</sub> + ( v<sub>t</sub> ∗ sin( eψ<sub>t</sub> ∗ d<sub>t</sub> )*
* *eψ<sub>t+1</sub> = ψ<sub>t</sub> − ψdes<sub>t</sub> + ( v<sub>t</sub> / L<sub>f</sub> ∗ δ<sub>t</sub> ∗ d<sub>t</sub> )*

### Cost function

The cost function has coefficients for position error and velocity as well as the steering and throttle actuators. 

## Loop

* Pass the current state as the initial state to the model predictive controller.
* Call the optimization solver. Given the initial state, the solver will return the vector of control inputs that minimizes the cost function.
* Apply the control input to the vehicle.

### Optimizer
The MPC uses an optimizer for the control inputs [δ<sub>1</sub>, a<sub>1</sub>, ..., δ<sub>N−1</sub>, a​<sub>N−1</sub>].  It finds locally optimal values while keeping the constraints defined by the non-actuator and actuator parameters.  

The optimizer used in this project is [Ipopt](https://projects.coin-or.org/Ipopt) (**I**nterior **P**oint **OPT**imizer, pronounced eye-pea-Opt), which works great for large-scale ​nonlinear optimization.

### Automatic Differentiation
Ipopt requires the jacobians and hessians directly -- it does not automatically compute them.  Therefore, this project uses [CppAD](https://www.coin-or.org/CppAD/) to compute the derivative values.  The AD (**A**utomatic **D**ifferentiation) returns the cost function and the approximated solution for steering angle and throttle (δ, a). 

# Adjusting for Latency

A contributing factor to latency is actuator dynamics.  For example, the time elapsed between when a new angle is sent to the steering actuator to when that angle is actually achieved. This is modeled in the MPC project as a 100ms latency between when the MPC process completes to when the actuator data is passed back into the system.

The speed at which data messages are sent to the program is highly influenced by the resolution and graphics quality selected in the opening screen of the simulator.  Other factors include speed of the machine running the simulator, the OS and if other programs are competing for CPU/GPU usage.  While these factors greatly influenced the perfomance of the PID controller, they became irrelevant for MPC once the artificial 100ms latency was added.


# Tuning the Hyperparameters


# Tracking

Tracking all the data points throughout the process of developing the MPC required a dedicated **Tracker class**.  This class collated all the pertinent data and reported it in periodic output like the following:  

```
best_cte_=0.00069, worst_cte_=2.90097, ave_cte_=0.52483
best_cost_=0, worst_cost_=563070, ave_cost_=204308
best_speed_=101.98, ave_speed_=75.53, ave_throttle_=0.845, ave_tps_=7.6
```

In addition to the periodic tracking from above, the waypoints provide a mechanism to track actual **laps**.  This is relevent data to capture because racing is based on lap times, not highest speed.  Also, if a twiddle algorithm is used for tuning hyperparameters, the error rate per-lap could be used instead of some arbitrary sample size. 

Lap tracking example:
```
num_laps_=3, best_lap_=31.92s, ave_lap_=33.53s
```


# Results
After much tinkering, my implementation runs well enough to get to 101 MPH and stays on the track.

[Link to project video](https://youtu.be/RKa5MHXwHRo)

# Lessons Learned

It would be interesting to try a **twiddle algorithm** to tune specific lap-based data-points (like CTE or lap time).  It could be used to tune the coefficient values of the cost function.

