source "./camera_utils.m"

% It returns a vector containing an initial guess on the position of the landmarks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% mode:        it is a string that represent the type of weight used to
%%              compensate for the cumulative error of the pose
%%              possible values are: ['no_weight', 'linear', 'hyperbola', 'log']
%%
%% landmarks:   is a num_landmarks x 3 array
%%              each row contains a 3D_position
function landmarks = get_initial_guess(mode='log')

    [As, bs, Ws] = explore_measurements();
    landmarks = solve_system(As, bs, Ws, mode=mode);
end

% Explore measurements to gather triangulation equations.
% In particular, for each landmark, stack together all the triangulation
% equations coming from observing the same landmark from different poses
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% [As, bs, Ws]: As * p = bs
%% As:          Cell array containing triangulation matrices for each landmark
%% bs:          Cell array containing triangulation vectors for each landmark
%% Ws:          Cell array containing weight vector for each landmark
function [As, bs, Ws] = explore_measurements()
    global num_poses num_landmarks traj observations;

    As = cell(num_landmarks, 1);
    bs = cell(num_landmarks, 1);
    Ws = cell(num_landmarks, 1);

    for i = 1:num_poses
        robot_pose = from_3dv_to_3dt(traj(i, 1:3));
        ob = observations{i};
        for j=1:length(ob)
            l_id = ob(j, 1);
            z = ob(j, 2:3)';
            [A, b] = get_triangulation_equations(robot_pose, z);

            As{l_id+1} = [As{l_id+1}; A];
            bs{l_id+1} = [bs{l_id+1}; b];
            Ws{l_id+1} = [Ws{l_id+1}; i; i];
        end
    end

end

% Solves a system of equations to return the triangulated 3D positions for all points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% As:          Cell array containing triangulation matrices for each landmark
%% bs:          Cell array containing triangulation vectors for each landmark
%% Ws:          Cell array containing weight vector for each landmark
%% mode:        it is a string that represent the type of weight used to
%%              compensate for the cumulative error of the pose
%%              possible values are: ['no_weight', 'linear', 'hyperbola', 'log']
%%
%% points:      is a num_landmarks x 3 array
%%              each row contains the triangulated 3D_position of the landmark
function points = solve_system(As, bs, Ws, mode=mode)
    global num_poses num_landmarks;
    points = ones(3, num_landmarks)*(-1);

    for i = 1:num_landmarks
        % if we have at least two correspondances we can triangulate
        if (length(bs{i}) > 2)
            switch mode
                case 'no_weight'
                    w = ones(1, length(Ws{i}));
                case 'linear'
                    w = 1 - num_poses+1 ./ Ws{i};
                case 'hyperbola'
                    w = 1 ./ Ws{i};
                case 'log'
                    w = 1 - (log(Ws{i}) ./ log(num_poses+1));
                otherwise
                    w = 1 - (log(Ws{i}) ./ log(num_poses+1));
            end
            
            % x = lscov(A,b,w) returns x that minimizes r'*diag(w)*r, where r = b - A*x.
            points(:, i) = lscov(As{i}, -bs{i}, w);
            %points{i} = -As{i}\(bs{i});
        end
    end
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
    w2c = world_to_camera(robot_pose);

    A = [w2c(1, 1:3) - p_img(1) * w2c(3, 1:3);
         w2c(2, 1:3) - p_img(2) * w2c(3, 1:3)];

    b = [w2c(1, 4) - w2c(3, 4) * p_img(1);
         w2c(2, 4) - w2c(3, 4) * p_img(2)];
end