global COLORS
COLORS.RED = "#e829a8";
COLORS.PURPLE = "#7e2f8e";
COLORS.GREEN = "#65e24b";
COLORS.BLUE = "#4b65e2";
COLORS.YELLOW = "#eace9d";

function _ = plot_trajectories(XR, initial_odom, gt_poses)
    global COLORS
    h = figure(1);
    
    subplot(1, 2, 1);
    % Plot the trajectory from the ground truth poses
    plot(gt_poses(:, 1),gt_poses(:, 2), 'color', COLORS.GREEN, 'linewidth', 2);
    hold on;
    % Plot the trajectory from the odometry poses
    plot(initial_odom(:,1), initial_odom(:,2), 'color', COLORS.PURPLE, 'linewidth', 2);
    grid
    daspect([1 1 1])
    title('Initial data')
    legend('ground truth', 'odometry')
    
    subplot(1, 2, 2);
    % Plot the trajectory from the ground truth poses
    plot(gt_poses(:, 1),gt_poses(:, 2), 'color', COLORS.GREEN, 'linewidth', 2);
    hold on;
    % Plot the trajectory from the optimized poses
    plot(XR(:, 1),XR(:, 2), 'color', COLORS.BLUE, 'linewidth', 2);
    grid
    daspect([1 1 1])
    title('After optimization')
    legend('ground truth', 'optimized')

    %waitfor(h);
end

function _ = plot_landmarks(XL, initial_guess, gt_landmarks)
    global COLORS
    h = figure(2);

    subplot(2, 2, 1);
    % Plot the ground truth landmarks
    plot3(gt_landmarks(1, :),gt_landmarks(2, :),gt_landmarks(3, :), '+', 'color', COLORS.GREEN);
    hold on;
    % Plot the initial guess of landmarks
    plot3(initial_guess(1, :),initial_guess(2, :),initial_guess(3, :),'o', 'color', COLORS.PURPLE);
    grid
    daspect([1 1 1])
    title('Initial data')
    legend('ground truth', 'initial guess')
    
    subplot(2, 2, 2);
    % Plot the ground truth landmarks
    plot3(gt_landmarks(1, :),gt_landmarks(2, :),gt_landmarks(3, :), '+', 'color', COLORS.GREEN);
    hold on;
    % Plot the optimized landmarks
    plot3(XL(1, :),XL(2, :),XL(3, :),'o', 'color', COLORS.BLUE);
    grid
    daspect([1 1 1])
    title('After optimization')
    legend('ground truth', 'optimized')

    subplot(2, 2, 3);
    % Plot the ground truth landmarks
    plot3(gt_landmarks(1, :),gt_landmarks(2, :),gt_landmarks(3, :), '+', 'color', COLORS.GREEN);
    hold on;
    % Plot the initial guess of landmarks
    plot3(initial_guess(1, :),initial_guess(2, :),initial_guess(3, :),'o', 'color', COLORS.PURPLE,"linewidth",2);
    grid
    view(-32, 17)
    xlim([-10, 10])
    ylim([-10, 10])
    zlim([0, 2])
    title('Close up')

    subplot(2, 2, 4);
    % Plot the ground truth landmarks
    plot3(gt_landmarks(1, :),gt_landmarks(2, :),gt_landmarks(3, :), '+', 'color', COLORS.GREEN);
    hold on;
    % Plot the optimized landmarks
    plot3(XL(1, :),XL(2, :),XL(3, :),'o', 'color', COLORS.BLUE,"linewidth",2);
    grid
    view(-32, 17)
    xlim([-10, 10])
    ylim([-10, 10])
    zlim([0, 2])
    title('Close up')

    %waitfor(h);
end

function _ = plot_state(XR, initial_odom, gt_poses, XL, initial_guess, gt_landmarks)
    % 2D Trajectories
    plot_trajectories(XR, initial_odom, gt_poses)
    
    % % 3D Landmark Positions
    plot_landmarks(XL, initial_guess, gt_landmarks)
end

function _ = plot_statistics(chi_stats, inliers_stats)
    global COLORS

    h = figure(3);

    % Chi Evolution    
    subplot(2, 2, 1)
    plot(chi_stats(1, :), 'color', COLORS.RED, 'linewidth', 2);
    title('chi evolution')
    grid
    legend("Chi Landmark")

    subplot(2, 2, 3)
    plot(chi_stats(2, :), 'color', COLORS.RED, 'linewidth', 2);
    grid
    legend("Chi Poses")
    xlabel("iterations")

    % Inliers Evolution 
    subplot(2, 2, 2)
    plot(inliers_stats(1, :), 'color', COLORS.BLUE, 'linewidth', 2);
    title('inliers evolution')
    grid
    legend("#inliers Landmark", 'location', 'southeast')

    subplot(2, 2, 4)
    plot(inliers_stats(2, :), 'color', COLORS.BLUE, 'linewidth', 2);
    grid
    legend("#inliers Poses")
    xlabel("iterations")

    waitfor(h);
end

function _ = plot_RMSE(rmse_stats)
    % RMSE    
    h = figure(4);
    subplot(1, 3, 1)
    plot(rmse_stats(1, :), 'color', COLORS.YELLOW, 'linewidth', 2);
    xticks([])
    title('RMSE poses. Rotation')

    subplot(1, 3, 2)
    plot(rmse_stats(2, :), 'color', COLORS.YELLOW, 'linewidth', 2);
    xticks([])
    title('RMSE poses. Translation')

    subplot(1, 3, 3)
    plot(rmse_stats(3, :), 'color', COLORS.YELLOW, 'linewidth', 2);
    xticks([])
    title('RMSE landmarks')

    waitfor(h);
end