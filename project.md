# #3 Planar Monocular SLAM

The aim of this project is to develop a SLAM system, i.e. to estimate both the trajectory of a robot and the position of a set of 3D points in the environment (landmarks).
A good estimate minimizes the error with respect to the given ground truth.
Thus, this repo contains the code of such SLAM system, some result images and error values, as a way to evaluate the quality of the system.

In particular, the robot is a Differential Drive robot, equipped with a monocular camera (with known extrinsics and intrinsics parameters) through which the robot, while moving on a plane, senses a set of 3D landmarks, each identified by a unique "id".
For each pose, a stream of point projections (of the landmarks on the image plane) with corresponding “id” are given.

The initial guess of the trajectory is provided as wheeled odometry, while [the next section](#initial-guess) describes how an initial guess for the position of the landmarks has been obtained.
Then, the approach chosen to improve such initial guess is a total least squares method, with pose-landmark constraints given by the point projections.
**[TODO aggiungere pose-pose constraints]()**

The project has been developed in OCTAVE and **[TODO RESULTS]()**

## Initial Guess

The 3D position of each landmark in the world was initialized by using the noisy poses of the robot and the points projected on the image planes.

I've used the triangulation method as described in [[1]](#1).
In particular, I got $p=[p_x \quad p_y \quad p_z]^T$ (with respect to the world frame) for each landmark measured at least twice.
The computation starts from the normalized coordinates $s_1=[\text{col}_1 \quad \text{row}_1]^T$, $\cdots$, $s_M=[\text{col}_M \quad \text{row}_M]^T$ of the projections of $p$ on the image planes of the M cameras corresponding to the M robot poses where the landmark is visible (each measurement).

Both intrinsic and extrinsic parameters of all the cameras are known.
Indeed, the transformation matrix from the world frame to the camera frame associated with the robot pose $n$ can be computed as:

$
O_n = \widetilde{K} \cdot \left ( \text{robot\_pose}_n \cdot \text{camera\_robot\_pose} \right )^{-1}
$

The parametric equation, expressed in the world frame, of the visual ray passing through $O_n$ and the point of coordinates $s_m$ is:

$p = o_n + \lambda_m \cdot R_n \cdot \widetilde{s_m} \quad$
where $o_n$ and $R_n = [r_1 \quad r_2 \quad r_3]$ are, respectively, the transitional and the rotational part of $O_n$

Therefore, we obtain the following system:

$
\begin{bmatrix}
r_1^T - \text{col}_m \cdot r_3^T\\ 
r_2^T - \text{row}_m \cdot r_3^T
\end{bmatrix}
p = 
\begin{bmatrix}
o_x - o_z \cdot \text{col}_m\\ 
o_y - o_z \cdot \text{row}_m
\end{bmatrix}
$

$A_m \cdot p = b_m\quad$ with $A$ is a 2x3 matrix and $b$ is a 2x1 vector.

by:
- premultiplying both sides by $R_n^T$
- setting $R_n^T \cdot o_n = [o_x \quad o_y \quad o_z]^T$
- computing $\lambda_m $ in the third equation and replacing its value into the other two equations

I computed $A_m$ and $b_m$ for each measurement of each landmark and then stacked together for each landmark in the form $A = [A_1 \cdots A_M]^T$ and $b = [b_1 \cdots b_M]^T$.

In the ideal case, all visual rays intersect at point $P$ and thus, of all these equation, only 3 are independent.
However, in practical applications like ours, because of noise, these equations are all independent and the system has no solution; hence, I computed the (approximated) least-squares solution.

Only 887 landmarks were observed out of 1000.
Moreover, out of those 887, only 838 were observed at least twice.

TODO: QUI SCRIVO CHE HO PROVATO CON LA WEIGHTED PER GIOCARE E METTO LE COMPARAZIONI/STATISTICHE DELLE VARIE FUNZIONI WRT LA GROUND TRUTH
TODO: POI GLI DICO CHE NEL DATASET CI SONO 6 PUNTI FALLATI PERCHE' HANNO LA Z TROPPO GRANDE
19631 measurements in total

It is interesting to note that, among the 19631 measurements in total in the dataset, there are 6 for which the landmark it is not visible in the camera, while measurements are present anyway.
In particular, all those landmarks are farther than the maximum camera depth.

# Least Squares

Once we had an initial estimate of 3D landmarks and odometry SE(2) poses, Bundle Adjustment was performed.

## State

Since I want the trajectory (sequence of poses) and the position of the 3D points, the state is composed by:
- N(=200 in my dataset) robot poses, each as a 3x3 homogeneous transform matrix since the motion of the robot is planar, thus SE(2) is enough
- M(=1000 in my dataset) landmark positions, each as a 3x1 array

#### Increments

The increments are composed by the minimal perturbation for each state variable, so defined:
- N minimal perturbations for the poses of the robot, each as a 3x1 array containing the increments of $x, y$ and $\theta$
- M minimal perturbation for the landmarks, each as a 3x1 array containing the increments of $x, y$ and $z$

Thus, the final perturbation vector is a [(N $\cdot$ 3)+(M $\cdot$ 3)]x1 array.

#### BoxPlus

The perturbations are applied to each variable block:
- BoxPlus operator for robot poses is defined as $X_r \boxplus x_r =  v2t(\Delta x_r) X_r$
- BoxPlus operator for landmark positions is defined as a classical sum $X_l \boxplus x_l = X_l + \Delta x_l$

## Pose-Landmark constraint
### Measurements

The measurements come from all sets of landmarks projected in the images.
Each robot pose is associated to one image, i.e. to one set of variable length (as in the measurement files) of measurements.
Each measurement is the 2x1 array of coordinates of the point in the image.

#### Predictions

A prediction for the robot pose $n$ and the landmark $m$ is the position of $X_l^{[m]}$ in the image associated with $X_r^{[n]}$.
It is obtained by first expressing the position of the landmark in the camera frame associated with $X_r^{[n]}$, and then project it into the image.

Thus, considering the camera matrix as $K$ and the pose of the camera with respect to the robot as $T_{\text{cam}}$, we have:

$
h^{[n,m]} = \text{proj} \left(K \cdot \left ( X_r^{[n]} \cdot T_{\text{cam}} \right )^{-1} \cdot X_l^{[m]} \right )
$

Moreover, to have a valid prediction, the point has to be inside the viewing frustum, thus additional checks are performed.

#### Error

Consequently, the error is simply the difference between predictions and measurements.
Since they are both 2D positions, we don't have to define a BoxMinus operator.

$e^{[n,m]}(X) =  h^{[n,m]}-z^{[n,m]}$

#### Jacobian

Since the prediction depends only on the robot pose $n$ and the landmark $m$, the entries of the Jacobian will be mostly 0.
It will be non 0 for the pose block $n$, i.e. $J_r^{[n,m]}$ and for the landmark block $m$, i.e. $J_l^{[n,m]}$.

As usual, we can expand those blocks:

$
J_r^{[n,m]} = \frac{\partial{e^{[n,m]}(X \boxplus \Delta x)}}{\partial{\Delta x_r^n}} = J_{\text{proj}} \cdot K \cdot J_{\text{icp}}
$

$
J_l^{[n,m]} = \frac{\partial{e^{[n,m]}(X \boxplus \Delta x)}}{\partial{\Delta x_l^m}} = J_{\text{proj}} \cdot K \cdot R_t
$

where, $R_t$ is the rotational part of the matrix $\left ( x_r^{[n]} \cdot T_{\text{cam}} \right )^{-1}$;

$
J_{\text{proj}} =
\begin{bmatrix}
1/z & 0  & -x/z^2 \\ 
0 & 1/z & -y/z^2
\end{bmatrix}
\quad $ where $x, y$ and $z$ are the coordinates of the landmark expressed in camera frame.

$
J_{\text{icp}} = 
\begin{bmatrix}
-R_t \cdot \begin{bmatrix}
-1 & 0 & y \\ 
0 & -1 & -x \\ 
0 & 0 & 0
\end{bmatrix}
\end{bmatrix}
\quad $ where $x$ and $y$ are the coordinates of the landmark expressed in world frame.

Lastly, the dimensions are:

$
J_r^{[n,m]} \in 2 \times 3 \\
J_l^{[n,m]} \in 2 \times 3 \\
J^{[n,m]} \in 2 \times (3 \cdot N + 3 \cdot M)
$

## Pose-Pose constraint
Several ways for implementing pose-pose constraint are available.
In particular, I've tried two methods.

The first is the Chordal Distance, as discussed in the slides of the course.
The error in this case is a $6 \times 1$ vector containing the differences between each element (4 rotational and 2 translational ones) of the two relative motions (estimated and measured).
The Jacobian is computed consequently.

The second method is the chosen one, i.e. the one that produce the results in the graphs.
It is described in the following.

### Measurements
We don't have direct measurements that relates subsequent poses.
However, we can retrieve them by using the (noisy) odometry.
Indeed, the relative motion between two subsequent poses $T_0, T_1$ can be measured as:
$rel_T = T_0^{-1} \cdot T_1$

Therefore, the measurements for adding a pose-pose constraint were retrieved by computing the relative motion from the odometry poses.

Then, I choose to use only the translational part of this relative motion, in particular computing the norm (magnitude) of the translation vector.
In other word, I considered the (scalar) euclidean distance of the displacement as measurement.

#### Predictions
The prediction is computed by computing the relative motion and then the norm of its translational part.

#### Error
The error in this case is a scalar containing the difference between the two computed norms (estimated and measured).

#### Jacobian
The Jacobian is computed by noting, as pointed out also in the slides, that given 
$
g^{[i, j]}(X) = X_r^{[i]-1}X_r^{[j]}
$
we have:

$
\left. \frac{\partial g\left ( X_i \boxplus \Delta x_i, X_j \boxplus \Delta x_j \right )}{\partial \Delta x_i} \right|_{\Delta x = 0} = - \left. \frac{\partial g\left ( X_i \boxplus \Delta x_i, X_j \boxplus \Delta x_j \right )}{\partial \Delta x_j} \right|_{\Delta x = 0}
$

Therefore we can focus on computing the derivative with respect to $\Delta x_j$, being the derivative with respect to $\Delta x_i$ only its opposite.

We have:

$
g(X_i, X_j \boxplus \Delta x_j) = X_i^{-1} v2t(\Delta x_j) X_j \\
\hspace{7.3em} =
\begin{bmatrix}
 R_i^T & -R_i^T t_i\\ 
 0 & 1
\end{bmatrix} 
\begin{bmatrix}
 R(\Delta \theta_j) & \Delta t_j\\ 
 0 & 1
\end{bmatrix}
\begin{bmatrix}
 R_j & t_j\\ 
 0 & 1
\end{bmatrix} \\
\hspace{7.3em} =
\begin{bmatrix}
 R_i^T R(\Delta \theta_j) R_j & R_i^T \left ( R(\Delta \theta_j) t_j + \Delta t_j - t_i \right )\\ 
 0 & 1
\end{bmatrix}
$

The norm of the translational part is computed by simply squaring and then taking the square root of the sum of the two components of the array, thus the derivatives are the following:

$
\left. \frac{\partial h\left ( X_i, X_j \boxplus \Delta x_j \right )}{\partial \Delta t_j} \right|_{\Delta x = 0} = 
\begin{bmatrix}
 \cos(\theta_i) - \sin(\theta_i) & \qquad
 \sin(\theta_i) + \cos(\theta_i)
\end{bmatrix}
\\
\left. \frac{\partial h\left ( X_i, X_j \boxplus \Delta x_j \right )}{\partial \Delta \theta_j} \right|_{\Delta x = 0} = 
\begin{bmatrix}
 \left ( \sin(\theta_i) + \cos(\theta_i) \right ) x_j -
 \left ( \cos(\theta_i) - \sin(\theta_i) \right ) y_j
\end{bmatrix}
$

The jacobian with respect to $\Delta x_i$ is the opposite.

## Results and evaluation

## Future works

# References
<a id="1">[1]</a>  Bruno Siciliano, Lorenzo Sciavicco, Luigi Villani, and Giuseppe Oriolo. 2010. Robotics: Modelling, Planning and Control. Springer Publishing Company, Incorporated.