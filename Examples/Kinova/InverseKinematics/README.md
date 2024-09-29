# Kinova Inverse Kinematics Example

`KinovaIKExample.cpp` provides the simplest example of solving an inverse kinematics problem on Kinova.
Both analytical gradient and hessian are provided to Ipopt.

Logic: 
1. randomly choose a configuration and use the FK to calculate the position and orienation of the end-effector
2. have initial guess of the start configuration and use IK to get the optimized trajectory

`KinovaIKMotionExample.cpp` provides an example of solving a series of inverse kinematics problems,
so that the end effector goes forward for 10 cm along its z axis.

```
desiredTransform = desiredTransform * Transform(Eigen::Vector3d(0, 0, step_size));

z = mynlp->solution;
```
Through this step, the end effector goes for 10 cm along its z axis in each iteration. 

Then it makes sense that the initial guesses of the configuration is the same as the wanted one.
```
    Eigen::VectorXd z = q;
```