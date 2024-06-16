source "./geometry_helpers.m"
source "./implementation_utils.m"
source "./visualisers.m"
source "./camera_utils.m"
source "./initial_guess.m"

function XR = fake_initialization_poses(traj)
    global num_poses
    XR = zeros(3, 3, num_poses);
    for i = 1:num_poses
        XR(:, :, i) = v2t(traj(i, 4:6)');
    end
end

function XL = fake_initialization_landmark(world_landmarks)
    XL = world_landmarks(:,:)';
end

function [XR, XL] = fake_initialization_status(traj, world_landmarks)
    XR = fake_initialization_poses(traj);
    XL = fake_initialization_landmark(world_landmarks);
end

function [XR, XL, chi_stats, inliers_stats, rmse_stats] = LS_without_noise(traj, world_landmarks, it, observations, landmark_associations, initial_odometry, kernel_threshold_land, kernel_threshold_pose)
    [XR, XL] = fake_initialization_status(traj, world_landmarks);
    gt_poses = XR;
    chi_stats = zeros(1, it);
    inliers_stats = zeros(1, it);
    rmse_stats = zeros(2, it);
    for k = 1:it
        [XR, XL, chi_tot, num_inliers, H, b] = doLS(XR, XL, observations, landmark_associations, initial_odometry, kernel_threshold_land, kernel_threshold_pose);
        chi_stats(1, k) = chi_tot;
        inliers_stats(1, k) = num_inliers;
        rmse_poses = compute_RMSE_poses(XR, gt_poses);
        rmse_landmarks = compute_RMSE_landmarks(XL, XL_true);
        rmse_stats(1, k) = rmse_poses;
        rmse_stats(2, k) = rmse_landmarks;
    end
end

function inside_distance_range = statistics_initial_guess(traj, world_landmarks, distance_range, observations, landmark_associations)
    global num_landmarks
    XL = zeros(3, num_landmarks);
    XL = get_initial_guess(traj(:, 1:3)', observations, landmark_associations);
    inside_distance_range = zeros(distance_range, 1);
    for j = 1:distance_range
        k = 0;
        for i = 1:num_landmarks
            if XL(:, i) ~= [ -1; -1; -1];
                gt = world_landmarks(i, :);
                
                if (norm(gt'-XL(:, i))<j)
                    k += 1;
                end
            end
        end
        inside_distance_range(j) = k;
    end

    inside_distance_range
end

function k = wrong_measurements(traj, world_landmarks, observations, landmark_associations)
    global num_poses num_landmarks;
    k = 0;
    ps = [];
    robot_poses = traj(:, 4:6)';

    for i = 1:num_landmarks
        gt = world_landmarks(i, :)';
        ob_indeces = find(landmark_associations(2, :)==i-1);
        ob = observations(:, ob_indeces);
        num_measurements = length(ob_indeces);
        pose_ids = landmark_associations(1, ob_indeces);

        for j=1:num_measurements
            pose_id = pose_ids(j);
            robot_pose = v2t((robot_poses(:, pose_id+1))');
            z = ob(:, j);
            [p_cam, visible1] = point_in_camera(gt, robot_pose);
            [p, visible2] = project_point(p_cam);
            if ((not(visible1)) || (not(visible2)) || (norm(z-p)>0.5))
                k += 1;
                ps = [ps [z; NaN; p_cam; NaN; p]];
            end
        end
    end
    ps
    k
end