1;
% The following functions are based on the loadG2o function, developed for
% the lectures of probabilistic robotics at Sapienza, University of Rome.
% Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti

function [world_landmarks, camera_infos, traj, observations] = load_all(num_poses, num_landmarks)
    world_landmarks = loadWorld(num_landmarks);
    camera_infos = loadCameraInfos();
    traj = loadTrajectories(num_poses);
    observations = loadMeasurements(num_poses);
end

% world.dat contains information about the map
% Every row contains: LANDMARK_ID (1) POSITION (2:4)
% Needed for the evaluation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% world_landmarks is a num_landmarks x 3 array
%% each row contains a 3D_position
%% the ID of the landmark is (row_index - 1)
function world_landmarks = loadWorld(num_landmarks)
    W=load("data/world.dat");
    world_landmarks = zeros(num_landmarks, 3);
    world_landmarks = W(:, 2:4);
end


% camera.dat contains information about the camera used to gather data:
% camera matrix
% cam_transform: pose of the camera w.r.t. robot
% z_near/z_far how close/far the camera can perceive stuff
% width/height of images
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% camera_infos is an object containing the following properties:
%% [K, T, z_near, z_far, width, height] with the meanings explained above
function camera_infos = loadCameraInfos()
    % Open the file
	fid = fopen("data/camera.dat", 'r');

    while true
		% Get current line
		c_line = fgetl(fid);

		% Stop if EOF
		if c_line == -1
            fclose(fid);
			break;
		end

		% Remove leading and trailing whitespace
        % And split the line using colon as separator
        c_line = strtrim(c_line);
		elements = strsplit(c_line,':');

        switch(elements{1})
            case 'camera matrix'
                camera_infos.K = extractMatrix(fid, 3);
            case 'cam_transform'
                camera_infos.T = extractMatrix(fid, 4);
            case 'z_near'
                camera_infos.z_near = str2double(elements{2});
            case 'z_far'
                camera_infos.z_far = str2double(elements{2});
            case 'width'
                camera_infos.width = str2double(elements{2});
            case 'height'
                camera_infos.height = str2double(elements{2});
            otherwise
                if not(strcmp(elements{1}, ""))
                    disp(['Error in reading element ', elements{1}, ...
                    'while reading file ', filepath]);
                end
                continue;
        end
    end
end

function out = extractMatrix(fid, lines)
    % For each line of the file
    for j = 1:lines
        % Remove leading and trailing whitespace
        % And split the line using space as separator
        c_line = strtrim(fgetl(fid));
        elements = strsplit(c_line,' ');
        % Convert each element to double
        % And add it to the matrix
        out(j, :) = arrayfun(@(x) str2double(x), elements);
    end
end


% trajectory.dat POSE_ID (1) ODOMETRY_POSE (2:4) GROUNDTRUTH_POSE (5:7)
% The ODOMETRY_POSE is obtained by adding Gaussian Noise (0; 0.001) to the actual robot commands
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% traj is a num_poses x 6 array
%% each row contains [odometry_pose gt_pose]
%% the ID of the pose is (row_index - 1)
function traj = loadTrajectories(num_poses)
    # Those are the same as the ones from measurement files
    T=load("data/trajectoy.dat");
    traj = zeros(num_poses, 6);
    traj = T(:, 2:7);
end


% meas-XXXXX.dat contains information about the XXXXX measurement
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% observations is a num_poses x 1 cell array
%% each cell contains a n x 3 array of observations with n variable
%% the ID of the pose is (row_index - 1)
function observations = loadMeasurements(num_poses)
    observations = cell(num_poses, 1);
    for i = 1:num_poses
        filepath = ["data/meas-", num2str(i-1, '%05.f'), ".dat"];
        points = loadMeasFile(filepath);
        observations{i} = points(:,:);
    end
end

% Every measurement contains a sequence number
% groundtruth (of the robot) and odometry pose
% and measurement information:
%  point POINT_ID_CURRENT_MESUREMENT (1) ACTUAL_POINT_ID (2) IMAGE_POINT (3:4)
% The Image_Point represents the pair [col;row] where the landmark is observed in the image
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% points is a n x 3 array with n = the actual number of measurements
%% each row contains [point_id col_of_image_point row_of_image_point]
%% the ID of the measurement is (row_index - 1)
function points = loadMeasFile(filepath)
    % Open the file
	fid = fopen(filepath, 'r');

    if fid == -1
        disp(['Error in reading file ', filepath]);
        return
    end

    points = [];
    while true
		% Get current line
		c_line = fgetl(fid);

		% Stop if EOF
		if c_line == -1
            fclose(fid);
			break;
		end

        % Remove leading and trailing whitespace
        % And split the line using space as separator
        c_line = strtrim(c_line);
		elements = strsplit(c_line,' ');

        switch(elements{1})
            case 'point'
                p = arrayfun(@(x) str2double(x), elements)(3:5);
                points = [points; p];
            case {'seq:', 'gt_pose:', 'odom_pose:'}
                % does nothing
            otherwise
                if not(strcmp(elements{1}, ""))
                    disp(['Error in reading element ', elements{1}, ...
                    'while reading file ', filepath]);
                end
                continue;
        end
    end
end
