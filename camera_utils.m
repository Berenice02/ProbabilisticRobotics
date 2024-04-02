1;

% find the position in the image of an observed point 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% camera_infos: is an object containing the informations about the camera:
%%               [K, T, z_near, z_far, width, height]
%% point:        is a 3x1 vector containing the position of the point wrt the world frame
%% robot_pose:   is a 3x3 transform matrix representing the pose of the robot
%%
%% p_img:        is a 2x1 vector containg the [col;row] of the point in the image
function p_img = project_point(camera_infos, point, robot_pose)
    % Initialization as an out-of-image point
    p_img = [-1; -1];

    % World to camera transform
    world_to_camera = robot_pose*camera_infos.T;
    % Project the point
    point_in_camera = world_to_camera(1:3, 1:3)'*(point-world_to_camera(1:3, 4));

    % Ignore it if it's too close or too far
    if ( point_in_camera(3) < camera_infos.z_near ||
         point_in_camera(3) > camera_infos.z_far )
        return;
    end

    % Express it with camera matrix and project
    projected_point = camera_infos.K * point_in_camera;
    p_img = projected_point(1:2)./projected_point(3);

    % Ignore it if it's outside the image boundaries
    if ( p_img(1) < 0 || p_img(1) > camera_infos.width ||
         p_img(2) < 0 || p_img(2) > camera_infos.height )
        return;
    end
end

