# PID Controller Reflection
PID Controller project reflection.


[image1]: ./video/p.png "Proportional Steering Angle"
[image2]: ./video/tuning_p_gain.png "Tuning P Gain"
[image3]: ./video/tuning_d_gain.png "Tuning D Gain"
[image4]: ./video/tuning_i_gain.png "Tuning I Gain"


## PID Controller
PID Controller is commonly used in industrial control systems. It can be used to control autonomus vehicle especially steering wheel.

In the PID Controller project main goal is to control the autonus vehicle in the simulator to circle the track at least once safely.  

## What is PID Controller
PID Controller allows us to smoothly control the system or vehicle using three hyper parameters.

### Error Value or CTE
PID Controller continously calculates error value, which is difference between desired value and measured value of essential system variable. In the context of autonomous vehicle this error value is called Cross Track Error (CTE), minimum distance between center of the track and vehicle position.

### Proportional Term
The first hyper parameter Kp allows us to tune proportional term. Proportional term steers vehicle rather than by fixed steering angle, proportional to the CTE. Furthur away from the center of the track, harder or larger the steering angle is. 
Kp * CTE

![Proportional Steering Angle][image1]
<br/>Image from video [1].

### Derivative Term
Though proportional term reduces CTE, it still over shoots center line. The second hyper parameter Kd allows us to tune over shooting by scaling CTE error rate, also called derivate term.
Kd * ( (CTE - CTE_prev) / deltaT)


### Integral Term
Proportional and Derivative terms help us reduce CTE but they can not reduce error due to vehicle/system's lane offset or steady state error. Scaled Integral term is used to offset error caused by steady state error.
CTE_cumulative = CTE_cumulative_prev + CTE
Ki * CTE_cumulative 

### Steering Angle
angle = (-Kp * CTE) + (-Kd * ( (CTE - CTE_prev) / deltaT)) + (-Ki * CTE_cumulative)

## Tuning PID Controller Hyper Parameter
There are many ways to tune the PID controller hyper parameters, Twiddle algorithm as mentioned in the class, tuning manually and even SGD (stochastic gradient descent).

### Twiddle
Tuning through Twiddle may be faster and accurate. But in our case of tuning simulation environment, coming up with initial parameter values and recovering vehicle from off-track position and adapting Twiddle for vehicle not completing pre-determined steps not only becomes critical but also time consuming.

### Manual Tuning
Started with zero values for Kp, Kd and Ki. 

### Tuning P Gain
First tuned the Kp to keep the vehile on the tack but with rapid oscillation or zig-zag motion remained.

![Tuning P Gain][image2]
<br/>Image from video [1].

### Tuning D Gain
Next tuned D gain to reduce oscillation and occasionally fine tuned P gain to check oscillation can be further reduced.

![Tuning D Gain][image3]
<br/>Image from video [1].

### Tuning I Gain
Finally tried to tune I gain, but did not have much effect may be simulate vehicle does not have much of steady state error.

![Tuning I Gain][image4]
<br/>Image from video [1].

### Final Hyper Parameter Values
Final hyper parameter values were as below with speed limit of 20.

```C++
    double kp = 45.0;
    double kd = 950.0;
    double ki = 0.0008;   
```

</br>
[video1](./video/t2p4_15_3.mp4)
</br>

References:
</br>
[1]  https://www.youtube.com/watch?v=4Y7zG48uHRo&feature=youtu.be





