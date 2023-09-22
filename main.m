close all
clear
clc

#import utils
source "./loaders.m"
source "./utils.m"
source "./visualisers.m"

[world_landmark, camera_infos, traj, gt_poses, odom_poses, observations] = loaders();

plot_trajectories(traj);