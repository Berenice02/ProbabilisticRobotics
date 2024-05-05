1;

% get the world to camera transformation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% robot_pose:   is a 3x3 transform matrix representing the pose of the robot
%%
%% w2c:          is a 4x4 transform matrix representing the world to camera transformation
function w2c = world_to_camera(robot_pose)
    global camera_infos;
    k = eye(4);
    k(1:3, 1:3) = camera_infos.K;

    w2c = inv(robot_pose*camera_infos.T);
    w2c = k*w2c;
end

% get the position of the point wrt camera
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% point:        is a 3x1 vector containing the position of the point wrt the world frame
%% robot_pose:   is a 3x3 transform matrix representing the pose of the robot
%%
%% p_cam:        is a 3x1 vector containing the position of the point wrt the camera origin/frame
%% visible:      is a boolean that expresses if the point is visible in the camera frame
function [p_cam, visible] = point_in_camera(point, robot_pose)
    global camera_infos;
    visible = true;

    % Get the world to camera transformation
    w2c = world_to_camera(robot_pose);

    p_cam = w2c*[point;1];
    p_cam = p_cam(1:3);

    % Ignore it if it's too close or too far
    if ( p_cam(3) < camera_infos.z_near ||
         p_cam(3) > camera_infos.z_far )
        visible = false;
    end
end

% find the position in the image of an observed point 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% p_cam:        is a 3x1 vector containing the position of the point wrt the camera frame
%%
%% p_img:        is a 2x1 vector containg the [col;row] of the point in the image
%% visible:      is a boolean that expresses if the point is visible in the image
function [p_img, visible] = project_point(p_cam)
    global camera_infos;
    visible = true;

    p_img = p_cam(1:2)./p_cam(3);

    % Ignore it if it's outside the image boundaries
    if ( p_img(1) < 0 || p_img(1) > camera_infos.width ||
         p_img(2) < 0 || p_img(2) > camera_infos.height )
        visible = false;
    end
end
