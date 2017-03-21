# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Configuring the sensor and motion models

The code has been modified so that all the problem-specific information, such as the motion models,
the sensor models and the associated covariance matrices are defined in the `main.cpp` file. 
The `ExtendedKalmanFilter` and `FusionEKF` classes merely use these to perform the filtering. New methods have been added to the `FusionEKF` class to allow the definition of sensors. 

Lines 18-153 of `main.cpp` should give a good example of how to use these methods. 