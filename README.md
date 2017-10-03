[//]: # (Image References)

[image1]: ./images/MPC_Result.jpg "MPC Result"

# Model Predictive Control for Autonomous Driving

Self-Driving Car Engineer Nanodegree Program

---

This project implement Model Predictive Control in C++ that automatically drive a vehicle smoothly at high speed in simulated environment. The program/application receives live data about the vehicle current position, steer direction, and speed. The vehicle information is fetch through a model that will calculate the optimal driving path and driving commands(actuators), which the application will send back to the vehicle to control/update the vehicle acceleration and steering. Then, the process is repeat with vehicle send back latest information to the application for another model iteration.

## Model Implementation

### State
The vehicle's state is respresent as a vector consist of the car's position in "x" and "y" coordinate in meters, heading direction notated as "ψ" (psi) in radian value, speed/velocity "v" in meters/second, cross track error "cte" in meter, and heading error e\psi (radians) at time t.

### Actuators 
Steering angle, acceleration, and brake are actuators that being modeled and store as vector. For simplicity acceleration and brake are combine as single actuator of throttle with range value of 1 and -1, which -1 produce a complete stop/brake. Steering is noted as delta and throttle is noted as "a" through out the codes.  

To model the physical constraint of the real world, steering is limited to turning angel of 25° and -25°.


### Update 
The current state will be passed to following equations calculate vehicle future state. 
```c++
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

## Polynomial Fitting and Preprocessing

The vehicle's position and traveling path data are passed to the application as world's coordinate perspective. The application must convert the given world's coordinate to the car's coordinate system/perspective, then pass these converted coordinate values to Polynomial fitting function.

Coordinates conversion is implemented on "main.cpp" between line 121 to 131.

## Timestep Length and Elapsed Duration

The application is also rely on two optimal parameters "N" and "dt", which are manualy set and tune. "N" repesent number of forward traveling points to calculate/predict, in other words, number of timesteps in the horizon. "dt" represent time delay factor that occur betwen each control actuation. 

The product result of "N" and "dt" will far(in time) in future horizon the application should calculate. Optimally this value should be a few a seconds at most.

Initially, "N" was set at low value of 5 while "dt" was set at high value of 0.5. But, these values produced undesirable result. "N" was finally tuned at 10 and "dt" was at 0.1. 

"N" and "dt" are set on "MPC.pp" at line 9 and 10;

## Final Result


[![Final Result][image1]](http://www.youtube.com/watch?v=ETqwuQWn_E8)

http://www.youtube.com/watch?v=ETqwuQWn_E8

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)





