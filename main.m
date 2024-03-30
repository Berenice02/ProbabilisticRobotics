close all
clear all
clc

#import utils
source "./loaders.m"
source "./utils.m"
source "./visualisers.m"
source "./camera_utils.m"

num_poses = 200;
num_landmarks = 1000;

[world_landmarks, camera_infos, traj, observations] = load_all(num_poses, num_landmarks);
odom_poses = traj(:, 1:3);

format long
% plot_odom_gt_trajectories_from_traj(traj);

% The state is composed by:
% - N robot poses, each as an array of 3x3 homogeneous transform matrices
% - M landmark positions, each as a 3x1 array
% XR=zeros(3, 3, num_poses);
% XL=zeros(3, num_landmarks);

% # Initialization
% XR(:, :, 1) = eye(3);
% for i = 2:num_poses
%     odom_T = v2t(odom_poses(i-1, :)');
%     XR(:, :, i) = XR(:, :, i-1) * odom_T;
% end

% landmark_associations=zeros(2,num_landmarks);
