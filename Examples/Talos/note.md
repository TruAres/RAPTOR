# Note
```
enum TimeDiscretization {
    Uniform = 0, 
    Chebyshev
};
```
Timediscretization represents different methods or strategies for discretizing time. 

Unlike uniform discretization, the steps may not be evenly spaced and are often clustered near the endpoints of the interval.

```
N = config["N"].as<int>();
```
This code assesses the value of the key "N" in config(YAML node) and convertr it to an integer type.

```
DenseBase<Derived>::Zero(Index size)
{
  return Constant(size, Scalar(0));
}
```
creates a vector (or matrix) where each element is initialized to the constant value 0 

# Question
```
typedef struct GaitParameters_ {
    double eps_torso_angle = Utils::deg2rad(1.5); // 3 degrees
    double swingfoot_midstep_z_des = 0.10; // meters
    double swingfoot_begin_x_des = 0.00; // meters 
    double swingfoot_begin_y_des = -0.17; // meters
    double swingfoot_end_x_des = 0.00; // meters
    double swingfoot_end_y_des = -0.17; // meters
} GaitParameters;
```
how to get the initial values of the parameters?

```
Eigen::VectorXd z = Eigen::VectorXd::Zero((degree - 1) * NUM_INDEPENDENT_JOINTS + NUM_JOINTS + NUM_DEPENDENT_JOINTS);
```
Does it mean the size of the vector needed to represent the trajectory?