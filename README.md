# Model-Predictive-Control_SelfDrivingCar_Term2_P5

This repo contains the submissions and related material for Udacity "Self Driving Car" Nano degree program's Term 2 - Project 5, "Model Predictive Control"


[Click here for the output video](https://youtu.be/bS4IiL4WTgw)


![alt text](https://github.com/sulabhmatele/Model-Predictive-Control_SelfDrivingCar_Term2_P5/blob/master/data_readme/MPC_Sim.png)

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

### MPC solve and different helper methods:

* `MPC solve` uses 2 main libs to efficiently optimize the `steering angnle` (δ) and `throttle` (a):

   #### IPOPT
   Ipopt is the tool we use to optimize the control inputs [δ
   ​1
   ​​ ,a1,...,δN−1,a
   ​N−1
   ​​ ]. It's able to find locally optimal values (non-linear problem!) while keeping the constraints set directly to the actuators and the constraints defined by the vehicle model. Ipopt requires us the jacobians and hessians directly - it does not compute them for us. Hence, we need to either manually compute them or have a library do this for us. Luckily, there is a library called `CppAD` which does exactly this.

   #### CppAD
   CppAD is a library we use for automatic differentiation. By using CppAD we don't have to manually compute derivatives, which is tedious and prone to error.
   
   In order to use CppAD effectively, we have to use its types instead of regular double or std::vector types.
   
   Additionally math functions must be called from CppAD. Here's an example of calling pow:
   
   `CppAD::pow(x, 2);
   // instead of 
   pow(x, 2);`
   
   Luckily most elementary math operations are overloaded. So calling `*, +, -, /` will work as intended as long as it's called on `CppAD<double>` instead of `double`.
   
* The implementation for solver goes as following steps:

  + Defining the vector of sufficient length to accomodate, different state parameters for the number of predicting states (N).
  
    Here with 6 state params + 2 desired params its -> `n_vars = N * 6 + (N - 1) * 2` 
  
  + Defining the constraint vector to define contraints for the parameters to not exceed a certain value.
  
  + Defining the constraints upper and lower limits for state and output vectors.
  
    Upper and lower limits for different params discussed in `Rubric Points` section above.
  
  + Defining the `operator()` method which would be called by solver implicitly, to accommodate the different `cost` and `constraints` equations.
  
    Cost is the sum of cost to control `cte, epsi, velocity` plus controlling `sterring angle, throttle` plus controlling and avoiding sharp changes in `sterring angle, throttle` 
    and its calculated as:
    
    
            // The part of the cost based on the reference state.
            for (int t = 0; t < N; t++)
            {
                fg[0] += 2000 * CppAD::pow(vars[cte_start + t], 2);
                fg[0] += 2000 * CppAD::pow(vars[epsi_start + t], 2);
                fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
            }
    
            // Minimize the use of actuators.
            for (int t = 0; t < N - 1; t++)
            {
                fg[0] += CppAD::pow(vars[delta_start + t], 2);
                fg[0] += CppAD::pow(vars[a_start + t], 2);
            }
    
            // Minimize the value gap between sequential actuation, for smooth transition
            for (int t = 0; t < N - 2; t++)
            {
                fg[0] += 200 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
                fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
            }
            
  Here the multipliers used as `2000, 200, 10` are chosen based on the observed performance of the model.
  These parameters influence the way model calculates the cost and puts more weight to correcting or controlling the
  respective parameter. 
  
  + Model different parameters, using different kinematic equations:
  
             // The equations for the model:
             // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
             // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
             // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
             // v_[t+1] = v[t] + a[t] * dt
             // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
             // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

  
  + Processing the `result` and return to `main`. 
  
  
#### Timestep Length and Elapsed Duration (N & dt)

The values for `N` and `dt` used here are:

    N = 10
    dt = 0.1 // 100ms
    
Choosing the above values made a proper balance of selecting `N` enough big to have sufficient data for predicted points, and also not making unnecessary calculations at each timestamps.
Different other values which were tried were - `N` as - 20 and keeping `dt` as - 0.05, 0.2.

With other different combinations it was found that, the calculated way points were either oscillating or calculating the predicted path which was not following the desired path.


#### Polynomial Fitting and MPC Preprocessing
These points are discussed above.

#### Model Predictive Control with Latency
The `latency` is the time difference between the vehicle issues the command to different actuator, to the 
time when actuator really processes the command.

In the simulated environment, its modelled by providing delay of 100ms. Then modeling the latency in the model to consider and predict the optimum value for `steering angel` and 
`throttle`.

Latency is processed as:

    // Predict the state after the latency: By kinematic equations
    // predict state in 100ms
    double lat_dt = 0.1;

    px = px + v * cos(psi) * lat_dt;
    py = py + v * sin(psi) * lat_dt;

    // The equation is updated to match the simulator
    // expectation of reverse left and right
    psi = psi - v * (delta/mpc.Lf) * lat_dt;
    v = v + acceleration * lat_dt; 
    
