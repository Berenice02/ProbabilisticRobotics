source "./camera_utils.m"

% It returns a vector containing an initial guess on the position of the landmarks
% In particular, it explore measurements to gather triangulation equations
% coming from observing the same landmark from different poses and then solves
% the linear system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% robot_poses:           it is a 3 x num_poses array
%% observations:          it is a 2 x num_measurements array
%%                        num_measurements is AT MOST num_poses * num_landmarks
%% landmark_associations: it is a 2 x num_measurements array
%%                        each column contains the vector [ID of the pose; ID of the landmark]
%% mode:                  NOT USED. IT WAS JUST A TRY TO MAKE INITIALIZATION BETTER
%%                        BUT IT DIDN'T WORKED CORRECTLY
%%                        it is a string that represent the type of weight used to
%%                        compensate for the cumulative error of the pose
%%                        possible values are: ['no_weight', 'linear', 'hyperbola', 'log']
%%
%% landmarks:   is a 3 x num_landmarks array; each column contains a 3D_position
%%              [-1; -1; -1] if no initial guess
function landmarks = get_initial_guess(robot_poses, observations, landmark_associations, mode='no_weight')
    global num_poses num_landmarks;
    landmarks = zeros(3, num_landmarks);

    for i = 1:num_landmarks
        ob_indeces = find(landmark_associations(2, :)==i-1);
        num_measurements = length(ob_indeces);
        if num_measurements < 2
            continue;
        end
        As = zeros(num_measurements*2, 3);
        bs = zeros(num_measurements*2, 1);
        ob = observations(:, ob_indeces);
        Ws = landmark_associations(1, ob_indeces);

        for j=1:num_measurements
            pose_id = Ws(j);
            robot_pose = v2t((robot_poses(:, pose_id+1))');
            z = ob(:, j);
            [A, b] = get_triangulation_equations(robot_pose, z);

            start_index = j*2-1;
            As(start_index:start_index+1, :) = A(:, :);
            bs(start_index:start_index+1, :) = b(:, :);
        end

        point = solve_system(As, bs, Ws, mode=mode);
        landmarks(:, i) = point(1:3);
    end
end

% Solves a system of equations to return the triangulated 3D positions for all points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% As:          num_measurements*2 x 3 array containing triangulation matrices of each measurement
%% bs:          num_measurements*2 x 1 array containing triangulation vectors of each measurement
%% Ws:          num_measurements x 1 array containing pose_id of each measurement
%% mode:        NOT USED. IT WAS JUST A TRY TO MAKE INITIALIZATION BETTER
%%              BUT IT DIDN'T WORKED CORRECTLY
%%              it is a string that represent the type of weight used to
%%              compensate for the cumulative error of the pose
%%              possible values are: ['no_weight', 'linear', 'hyperbola', 'log']
%%
%% point:      is the triangulated 3D_position of the landmark
function point = solve_system(As, bs, Ws, mode=mode)
    global num_poses;
    switch mode
        case 'no_weight'
            w = ones(1, length(Ws));
        case 'linear'
            w = (num_poses+1) ./ Ws;
        case 'hyperbola'
            w = 1 ./ Ws;
        case 'log'
            w = 1 - (log(Ws+1) ./ log(num_poses+1));
        otherwise
            w = ones(1, length(Ws));
    end
    
    w = repelem(w, 2);
    % x = lscov(A,b,w) returns x that minimizes r'*diag(w)*r, where r = b - A*x.
    point = lscov(As, -bs, w);
    % point = -As\(bs);
end

% It derives the A matrix and b vector to find the 3D position of a point
% given one of its normalized image coordinates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% robot_pose:   is a 3x3 transform matrix representing the pose of the robot
%% p_img:        is a 2x1 vector containg the [col;row] of the point in the image
%%
%% [A, b]:       A * p = b
%% A:            a 2x3 matrix
%% b:            a 2x1 vector 
function [A, b] = get_triangulation_equations(robot_pose, p_img)
    w2i = world_to_image(robot_pose);

    A = [w2i(1, 1:3) - p_img(1) * w2i(3, 1:3);
         w2i(2, 1:3) - p_img(2) * w2i(3, 1:3)];

    b = [w2i(1, 4) - w2i(3, 4) * p_img(1);
         w2i(2, 4) - w2i(3, 4) * p_img(2)];
end
