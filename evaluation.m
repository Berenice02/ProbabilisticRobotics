1;

function rmse = RMSE(e)
    rmse = sqrt(mean(e.^2));
end

function rmse = compute_RMSE_poses(XR, gt_poses)
    global num_poses
    rmse = 0;
    for i = 1:num_poses-1
        rel_T = inv(XR(:, :, i)) * XR(:, :, i+1);
        rel_GT = inv(gt_poses(:, :, i)) * gt_poses(:, :, i+1);

        error_T = inv(rel_T) * rel_GT;

        R_e = atan2(error_T(2, 1), error_T(1, 1));
        rmse += R_e;

        t_e = error_T(1:2, 3);
        rmse += RMSE(t_e);
    end
end

function rmse = compute_RMSE_landmarks(XL, gt_land)
    global num_landmarks
    rmse = 0;
    for i = 1:num_landmarks
        e = XL(:, i) - gt_land(:, i); 
        rmse += RMSE(e);
    end
end