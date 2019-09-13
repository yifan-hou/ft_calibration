# ft_calibration
TODO(yifan) dump file instead of copy paste
Static calibration of robot tool weight and force-torque sensor offset.
Start from the current pose, the robot will rotate about world X and Y axis,
while measuring forces. The behavior is parameterized in config file.

Computes:
1. G: Weight of end-effector (all the parts that connect to FT sensor)
2. P: x,y,z of end-effector center of mass(COM) in tool frame with respect to FT sensor. ((0 0 0) is the center of FT sensor)
3. T_bar, F_bar: Constant offset of FT sensor reading.

Before running the code, you need to figure the orientation of FT sensor in robot toolframe, and adjust the function src/calibration.cpp/getToolFrameWrench accordingly.

Require package mlab-infra/egm.

## Math
We want to compute:
$P$: vector of COM w.r.t FT sensor, measured in robot tool frame.
$G$: $[0; 0; mg]$, vector of gravity
$\bar F, \bar_T$: FT sensor reading offset. Reading + offset = true value

We can collect data for:
$R$: orientation of FT sensor.  Assume to be the same as robot tool frame orientation. When $R=I$, tool frame is aligned with world frame.
$F$: Force reading of FT sensor.
$T$: Torque reading of FT sensor.

Equation One:
$$R^{-1}G=F+\bar F$$
Equation two:
$$P\times(F+\bar F)=T+\bar T$$

Now we can solve the problem. From equation one, we can solve the following linear system for $G$ and $\bar F$:
$$Ax=b$$,
where
$$A = [R_{-1}, -I_3]$$
$$x=[G; \bar F]$$
$$b=F$$

Next, from equation two, we can solve the following linear system for $P$ and $\bar T$:
$$Ax=b$$,
where
$$A = [cross(\bar F), -I_3]$$
$$x=[P; \bar T]$$
$$b=T$$

Here $cross(x)$ means the 3x3 matrix that represents left cross product with x:
cross(x) = [0, x(2), -x(1);
			-x(2), 0, x(0);
			x(1), -x(0), 0]

## How to use the calibration result
When there is external force $F_{actual}$ acting on the end effector, the reading of ft sensor is (in tool frame, not raw data)

$$F_R = F + F_{actual}$$

And we know $F = R^{-1}G - \bar F$, thus

$$F_{actual} = F_R - F = F_R + \bar F - R^{-1}G $$

Similarly, for torque we have
$$T_{actual}=T_R+\bar T-P\times(R^{-1}G)$$