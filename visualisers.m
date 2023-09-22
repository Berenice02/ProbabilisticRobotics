source "./utils.m"

function h = plot_trajectories(traj)
    # Initialize the figure
    h = figure(1);

    # Compute and plot the trajectory from the odometry poses
    odomTrajectory=compute_odometry_trajectory(traj(:,2:4));
    plot(odomTrajectory(:,1),odomTrajectory(:,2), 'r-', 'linewidth', 2);
    hold on;

    # Compute and plot the trajectory from the ground truth poses
    gtTrajectory=compute_odometry_trajectory(traj(:,5:7));
    plot(gtTrajectory(:,1),gtTrajectory(:,2), 'g-', 'linewidth', 2);
    hold off;

    legend('odometry','ground truth')

    waitfor(h);
end
