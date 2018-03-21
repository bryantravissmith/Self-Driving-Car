# Report

## PID: Proportional, Integral, and Differential

PID is a control system that controls an actuator based on an measurement.  In terms
of this project, it is the steering angle of the car based on the cross-track error (
perpendicular distance from ideal path).

The P controller sets the steering angle proportional to how much error there is.  If a
CTE of 5.0m leads to a steering angle of 0.1 rads, then a P controller would give a steering
angle of 0.05 rads for an CTE of 2.5M and 0.2 rads for a CTE of 10.0 m.

The I controller is to control for bias in the setting of the angle.   If the actuator
is set to 0.0 rads, but is actually 0.1 rads, the car will drift off course.  The I portion
of the controller will sum this error over time and respond by correcting the setting to compensate for the bias.

The  D portion of the controller responds to how quickly the CTE is changing.  If it is changing quickly,
the D will damp the change.   If the CTE has quick increase or decrease, the D will counter that.

The PD portion, some applications, can be thought of as damped oscillation, and we want smooth
transitions to the correct path in autonomous driving.

## Setting PID Coefficients

As stated above, we want smooth transitions for autonomous driving, so the D terms should
overwhelm the p term to create a  over-damped oscillator.  Finally, bias is a huge deal in
autonomous driving, so we want a large bias correction factor.   

Using this framework, and a scan of the size of the CTE, I started with an initial setting of
0.1, 1.0, 10.0 for Kp, Kd, and Ki.   This lead to angles above/minus 1, so I divided by 10.
This got the car around the track, and I turned a little bit by increase the D term to smooth over
turn on curve and increase the I term because it seems to still had to correct.  

I finally came to 0.01, 0.25, 2.0 for the Kp, Kd, and Ki coefficients.
