# Overview
This repository contains an implementation of a particle filter for localizing a vehicle in a 2D environment. The code was written for the **Kidnapped Vehicle** project for Udacity's Self-driving Car Nanodegree.

## Project Introduction
A car has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

This project implements a 2 dimensional particle filter in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter also gets observation and control data. 

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Particle Filter Theory
A particle filter estimates the location of an object by starting with a list of likely locations and increasing or decreasing the probability of each location using a Map and information about the observed by the object. 

Every location is represented by a particle that contains all state variables being estimated. Intially a particle filter intiailizes a swarm of such particles with random values for the state variables. It then finds the probability of each particle representing the true position by comparing the sensor observations received from the true (unknown) position with the predicted observations computed using the particle's state in a map.

The Particle Filter is a kind of Bayesian Filter which is similar to an **Unscented Kalman Filter(UKF)** but uses a set of random points to represent the belief about an object's location,as opposed to using a set of carefully chosen points around a Gaussian surface in the case of UKF.


## Particle Filter implementation
The core logic of the particle filter is contained in the file `particle_filte.cpp` while the code in `main.cpp` has the interface code for communicating with the simulator through uWebSockets.

## Inputs to the Particle Filter
The inputs to the particle filter is in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Performance
The particle filter was run with 1000 particles. The results are summarized below
1. **Accuracy**: The table below summarizes the final errors after 2,443 simulation steps. 

|Measurement | Error      |
|:----------:|:----------:|
| X position | 0.108 m    |
| Y position | 0.102 m    |
| Orientation| 0.004 rad  |


2. **Execution Speed**: A hard limit of 100s was specified in the project rubric. The simulation took only 49.16s to finish. The speed can be further improved by storing the location of the landmarks as a hashmap. The number of particles could also be dynamically adjusted depending on the accuracy of sensor data. 


