close all
clear all
clc

source "./loaders.m"
source "./geometry_helpers.m"
source "./implementation_utils.m"
source "./visualisers.m"
source "./camera_utils.m"
source "./initial_guess.m"
source "./evaluation.m"

source "./testing.m"

global num_poses = 200;
global num_landmarks = 1000;
% num_measurements = 19631;
global camera_infos;

[world_landmarks, camera_infos, traj, observations, landmark_associations] = load_all(num_poses, num_landmarks);
%format long;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  INIZIALIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The state is composed by:
% - N robot poses, each as an array of 3x3 homogeneous transform matrices
% - M landmark positions, each as a 3x1 array

XR = zeros(3, 3, num_poses);
gt_poses = zeros(3, 3, num_poses);
initial_odometry = zeros(3, 3, num_poses-1);

% Initialization
for i = 1:num_poses
    XR(:, :, i) = v2t(traj(i, 1:3)');
    gt_poses(:, :, i) = v2t(traj(i, 4:6)');

    if (i == 1)
        continue
    end

    initial_odometry(:, :, i-1) = inv(XR(:, :, i-1)) * XR(:, :, i);
end

XL = zeros(3, num_landmarks);
gt_landmarks = world_landmarks(:,:)';
initial_XL = get_initial_guess(traj(:, 1:3)', observations, landmark_associations);
XL = initial_XL;

initial_RMSE = zeros(1, 3);
initial_RMSE(1:2) = compute_RMSE_poses(XR, gt_poses);
initial_RMSE(3) = compute_RMSE_landmarks(XL, gt_landmarks);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  OPTIMIZATION LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

iterations = 5;
kernel_threshold_land = 1e4;
kernel_threshold_pose = 1e-1;
chi_stats = zeros(2, iterations);
inliers_stats = zeros(2, iterations);

for k = 1:iterations
    [XR, XL, chi_tot, num_inliers] = doLS(XR, XL, observations, landmark_associations, initial_odometry, kernel_threshold_land, kernel_threshold_pose);
    chi_stats(:, k) = chi_tot;
    inliers_stats(:, k) = num_inliers;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Save results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

final_RMSE = zeros(1, 3);
final_RMSE(1:2) = compute_RMSE_poses(XR, gt_poses);
final_RMSE(3) = compute_RMSE_landmarks(XL, gt_landmarks);

% Reshape optimized trajectory
traj_opt = zeros(num_poses, 7);
for i = 1:num_poses
    traj_opt(i, 1) = i;
    traj_opt(i, 2:4) = t2v(XR(:, :, i));
end
traj_opt(:, 5:7) = traj(:, 4:6);

% Reshape landmarks
land_to_be_saved = zeros(num_landmarks, 7);
for i = 1:num_landmarks
    land_to_be_saved(i, 1) = i;
end
land_to_be_saved(:, 2:7) = [XL' gt_landmarks'];

% Reshape stats
stats = zeros(iterations, 5);
for i = 1:iterations
    stats(i, 1) = i;
    stats(i, 2:3) = chi_stats(:, i)';
    stats(i, 4:5) = inliers_stats(:, i)';
end

% POSE_ID (1) OPTIMIZED_POSE (2:4) GROUNDTRUTH_POSE (5:7)
save('output/XR.dat', 'traj_opt');
% LANDMARK_ID (1) OPTIMIZED_POSITION (2:4) GROUNDTRUTH_POSITION (5:7)
save('output/XL.dat', 'land_to_be_saved');
% ITERATION (1) CHI_LANDMARK (2) CHI_POSES (3) # INLIERS_LANDMARK (4) # INLIERS_ POSES (5)
save('output/chi_and_inliers.dat', 'stats');
% Rotation poses error (1) Translation poses error (2) Translation landmarks error
save('output/initial_RMSE.dat', 'initial_RMSE');
save('output/final_RMSE.dat', 'final_RMSE');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  PLOT RESULTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot_state(traj_opt(:, 2:4), traj(:, 1:3), traj(:, 4:6), XL, initial_XL, gt_landmarks)
plot_statistics(chi_stats, inliers_stats)
%plot_RMSE(rmse_stats)
