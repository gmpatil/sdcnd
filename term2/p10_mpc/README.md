# CarND-Controls-MPC
Model Predictive Control (MPC) Project

---

## Compilation and Build
```
cd sdcnd/term2/p10_mpc/
mkdir build
make
```
## The Model
 As the name suggests MPC - Model Predictive Control, consists of the model which predicts reasonably accuratly future states given current state and actuator controls. 

 At high level MPC process has below main steps:
 - Build Dynamic or Kinematic Model
 - Tune the model parameters

 ### Kinematic Model
As mentioned in class, Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass.

This simplification reduces the accuracy of the models, but it also makes them more tractable.

At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics.

#### Model States and Actuators
Model has below states which tracks vehicle's position, orientation and velocity.
```
x - global X co-ordinate of the vehicle's position
y - global Y co-ordinate of the vehicle's position
psi - Vehicle's global orientation or yaw angle.
v - Velocity of the vehicle.

Derived state values
cte - Cross Track Error
epsi - Orientation Error

```

Actuators:
```
delta - Steering angle
a - acceleration and break.
```

States are updated using below equations.

[comment]: <> (This is a comment, it will not be included)
```
x_t+1 = x_t + v_t * cos(psi_t) * dt
y_t+1 = y_t + v_t * sin(psi_t) * dt
psi_t+1 = psi_t + v_t / Lf * delta * dt
v_t+1 = v_t + a_t * dt)
```
$$x_{t+1} = x_t +  v_t * cos(\psi_t) * dt$$
$$y_{t+1} = y_t +  v_t * sin(\psi_t) * dt$$
$$\psi_{t+1} = \psi_t + v_t/L_f * \delta * dt $$
$$ v_{t+1} = v_t + a_t * dt $$ 

Where subscript t and t+1 represent state at time t and t+1. 
<br/>
Lf is distance between the front and the center of gravity of the vehicle.

### Non-Linear Optimization and Tuning Parameters
Problem of identifying optimal actuators' value is problem of  nonlinear optimization. IPOPT (Interior Point Optimizer, pronounced ``Eye-Pea-Opt'') library solver function is used to find optiomal actuator values.

Parameters like number of steps, duration of each step, cost factors for CTE, ePsi, and change in actuator values are tuned iteratively.

```
double ref_v = 40;

double cost_factor_cte = 10;
double cost_factor_epsi = 20;
double cost_factor_steering = 20;
double cost_factor_accel = 20;
double cost_factor_steering_change = 20;
double cost_factor_accel_change = 20;

...

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += cost_factor_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += cost_factor_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += cost_factor_steering * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += cost_factor_accel * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += cost_factor_steering_change * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += cost_factor_accel_change * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```
## Timestep Length and Elapsed Duration 
Frequency at which actuators are applied and number of steps used to predict future states which are used to optimize current actuator values are critical parameters tuned. 

Number of steps N and duration between steps dt determines time duration model repeatedly used to predict future state values. 
Too long total time duration  (N * dt) as well as too low degrades accuracy of predictions.  In my environment 1.2 seconds of total duration found to be ideal. Duration of 100 ms or above between steps (dt) worked well as latency was set at 100 ms.

Kept dt above 100 ms and tuned the duration (N * dt) by altering N. Then dt is tuned further.

```
size_t N = 12;
double dt = 0.10;
```

## Polynomial Fitting and MPC Preprocessing
Way points (reference points) returned by simulator is first converted to from global co-ordinates to vehicle co-ordinates and then used to fit to 3rd degree polynomial. 

Below equations were used to convert to local/vehicle co-ordinates.
```
      x_diff = x_global[i] - x_car_pos;
      y_diff = y_global[i] - y_car_pos;

      x_car[i] = x_diff * cos(car_psi) + y_diff * sin(car_psi);
      y_car[i] = y_diff * cos(car_psi) - x_diff * sin(car_psi);
...

      // Fit car x and y coordinates way points to 3rd order polynomial
      Eigen::VectorXd coeffs = polyfit(ptsx_car_coords, ptsy_car_coords, 3);
      
```
Vehicle co-oridinates' origin is at the vehicle's position and direction it is moving is x-axis, left side is the y-axis.
Converting to vehicle's co-ordinates is helpful as x, y and psi values for initial states will be zeros.
Cross Track Error (CTE) is approximated to Y-intercept.
Orientation error (epsi) is angle between x-axis and tangent of the polymonial at x = 0.

```
    // cte at car position which origin. y intercept at zero or 
    // just coeffs[0]
    double cte = coeffs[0];
          
    // epsi
    double epsi = -atan(coeffs[1]);
          
    // initial state
    Eigen::VectorXd state(6);          
    // (x,y) = origin (0,0)
    state << 0, 0, 0, v, cte, epsi;

```

### MPI Processing/Solving Non-Linear optimization
In order to solve non-linear optimization problem, all current and future tate and actuators are treated as variables. 
Thus we get below number of variables.
```
// 6 state variables and 2 actuator variables
size_t n_vars = N * 6 + (N - 1) * 2;
```
Lower and upper bounds and constraints for these variables are set and optimal actuator values are obtained. See 'MPC::Solve' method in MPC.cpp.

## Model Predictive Control with Latency
I considered below methods to handle latency issue. 
1. Project state just after latency duration (100 ms) and feed the projected state to non-leanear Solver.
2. Use the actuator values from future time step closer to latency period 
3. Calculate mean actuator values of first few future time steps such that mean time step is closer to the latency period 

I got relatively better results with the option #3.
 
## Video 
Below is link to the video demonstrating vehicle successfully driving a lap around the track.

https://github.com/gmpatil/sdcnd/blob/master/term2/p10_mpc/video/t2p5_1.mp4




