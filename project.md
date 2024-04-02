# State
Since I want the trajectory (sequence of poses) and the position of the 3D points, the state is composed by:
- N(=200 in my dataset) robot poses, each as a 3x3 homogeneous transform matrix since the motion of the robot is planar, thus SE(2) is enough
- M(=1000 in my dataset) landmark positions, each as a 3x1 array

## Increments
The increments are composed by the minimal perturbation for each state variable, so defined:
- N minimal perturbations for the poses of the robot, each as a 3x1 array containing the increments of $x, y$ and $\theta$
- M minimal perturbation for the landmarks, each as a 3x1 array containing the increments of $x, y$ and $z$

## BoxPlus
The perturbations are applied to each variable block:
- BoxPlus operator for robot poses is defined as $v2t(\Delta x_r) X_r$
- BoxPlus operator for landmark positions is defined as a classical sum $X_l + \Delta x_l$

# Measurements