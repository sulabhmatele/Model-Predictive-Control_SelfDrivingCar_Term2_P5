# Model-Predictive-Control_SelfDrivingCar_Term2_P5

This repo contains the submissions and related material for Udacity "Self Driving Car" Nano degree program's Term 2 - Project 5, "Model Predictive Control"


## Rubric Points

### Different code params: 

* Maximum speed achieved is - 95 mph (Reference velocity set to 100 mph)

* Throttle param limits     -> (-1) to (+1)

   (Note: Considering accelerator and brake as single control, and -1 indicates full brake, and +1 indicates full throttle)

* Steering angle limits     -> (-25) to (+25)

* Number of predicted states (N) -> 10 (Represented by green line)

* Time difference between 2 samples (dt) -> 0.1 sec (100 ms) (Represented by dots on green line)

* Number of reference points processed -> 25 (Represented by yellow line)


### Implementation :

This code implements the "Model predictive controller", which controls the simulated vehicle's 'Throttle' and 'Steering Angle', the input from simulator are:

* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.

#### Steps followed:

* We process the way points to compensate the latency. (Discussed in later section)
* Then we convert the global coordinate way points from simulator to the vehicle coordinates, using:

   `xc = x * cos(theta) - y * sin(theta)`
   
   `yc = x * sin(theta) + y * cos(theta)`
   
   Here we assume that, the vehicle is on track, and to find out the error and to make the different between the ref point and vehicle position we subtract the simulator points by the current position of the vehicle.
   We also want the angle difference between the vehicle and reference path to be 0, so we diff the `psi` by zero, which also compensates the simulator limitation of being shifted right by 90 degree. (At angle -90)
   
* Then we calculate the `coeff` (coefficients) of the polynomial of transformed way points. 
Polyfit is adopted from [link](https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716)

* Then `cte` is calculated by evaluating the polynomial coeff and then deduct the y. 

* `epsi` which is the error angle between the desired and actual direction of car, is calculated by `-atan(coeffs[1])`

* By now we have our state model of 6 parameters:

  `[x, y, ψ, v, cte, eψ]`
  
* We pass the state model of 6 parameters and `coeff` to `MPC` solve, which then calculates and models the constraints, and returns the desired optimized `steering angle` and `throttle` values.

