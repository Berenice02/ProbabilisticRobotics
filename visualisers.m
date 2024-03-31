global COLORS
COLORS.RED = "#e829a8";
COLORS.PURPLE = "#7e2f8e";
COLORS.GREEN = "#65e24b";
COLORS.BLUE = "#4b65e2";
COLORS.YELLOW = "#eace9d";

function _ = plot_trajectories(traj)
    global COLORS
    % Plot the trajectory from the odometry poses
    plot(traj(:,1),traj(:,2), 'color', COLORS.PURPLE, 'linewidth', 2);
    hold on;

    % Plot the trajectory from the ground truth poses
    plot(traj(:,4),traj(:,5), 'color', COLORS.GREEN, 'linewidth', 2);
    hold off;

    % TODO add optimized
    legend('odometry','ground truth')
end

function _ = plot_landmarks(gt_l)
    global COLORS
    plot3(gt_l(:,1),gt_l(:,2),gt_l(:,3), '+', 'color', COLORS.GREEN);
    hold on;
    % plot3(XL_guess(:,1),XL_guess(:,2),XL_guess(:,3),'o', 'color', COLORS.PURPLE,"linewidth",2);

    %z= zeros(200,1);
    %plot3(traj(:,4),traj(:,5),z, '-', 'color', COLORS.PURPLE);

    % TODO add guess
    % TODO add optimized
    hold off;

    legend('ground truth');
end

function _ = plot_chi_stats()
    % subplot(3,2,1);
    % plot(chi_stats_r, 'r-', "linewidth", 2);
    % legend("Chi Poses"); grid; xlabel("iterations");
    % subplot(3,2,2);
    % plot(num_inliers_r, 'b-', "linewidth", 2);
    % legend("#inliers"); grid; xlabel("iterations");

    % subplot(3,2,3);
    % plot(chi_stats_l, 'r-', "linewidth", 2);
    % legend("Chi Landmark"); grid; xlabel("iterations");
    % subplot(3,2,4);
    % plot(num_inliers_l, 'b-', "linewidth", 2);
    % legend("#inliers"); grid; xlabel("iterations");

    % subplot(3,2,5);
    % plot(chi_stats_p, 'r-', "linewidth", 2);
    % legend("Chi Proj"); grid; xlabel("iterations");
    % subplot(3,2,6);
    % plot(num_inliers_p, 'b-', "linewidth", 2);
    % legend("#inliers");grid; xlabel("iterations");
end

function _ = plot_H()
    % H_ =  H./H;                      # NaN and 1 element
    % H_(isnan(H_))=0;                 # Nan to Zero
    % H_ = abs(ones(size(H_)) - H_);   # switch zero and one
    % H_ = flipud(H_);                 # switch rows
    % colormap(gray(64));
    % hold on;
    % image([0.5, size(H_,2)-0.5], [0.5, size(H_,1)-0.5], H_*64);
    % hold off;
end

function _ = plot_state(traj, world_landmarks)
    % 2D Trajectories
    h = figure(1);
    plot_trajectories(traj)
    grid;

    % 3D Landmark Positions
    h = figure(2);
    plot_landmarks(world_landmarks)
    grid;

    % Chi Evolution
    h = figure(3);
    plot_chi_stats()
    grid;

    % H Matrix
    h = figure(4);
    plot_H()

    waitfor(h);
end
