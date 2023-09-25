close all
clear
clc

#import utils
source "./loaders.m"
source "./utils.m"
source "./visualisers.m"
source "./camera_utils.m"

[world_landmark, camera_infos, traj, gt_poses, odom_poses, observations] = loaders();

# Those are the same
% plot_odom_gt_trajectories_from_traj(traj);
% plot_odom_gt_trajectories_from_poses(gt_poses, odom_poses);

traj = compute_odometry_trajectory(gt_poses(1:9, :));
robot_pose = from_2d_to_3d(traj(9,:));
dove_sta_nella_mappa = [6.80375; -2.11234;   1.1324];
p_img = project_point(camera_infos, dove_sta_nella_mappa, robot_pose)
