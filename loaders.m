1;
% The following functions are based on the loadG2o function, developed for
% the lectures of probabilistic robotics at Sapienza, University of Rome.
% Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti

function [world_landmarks, camera_infos, traj, observations, landmark_associations] = load_all(num_poses, num_landmarks)
    world_landmarks = loadWorld(num_landmarks);
    camera_infos = loadCameraInfos();
    traj = loadTrajectories(num_poses);
    [observations, landmark_associations] = loadMeasurements(num_poses, num_landmarks);
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
%% observations is a 2 x num_measurements array
%%      num_measurements is AT MOST num_poses * num_landmarks
%%      each column contains the 2 x 1 observation
%% landmark_associations is a 2 x num_measurements array
%%      each column contains the vector [ID of the pose; ID of the landmark]
function [observations, landmark_associations] = loadMeasurements(num_poses, num_landmarks)
    num_measurements = num_poses * num_landmarks;
    observations = zeros(2, num_measurements);
    landmark_associations = zeros(2, num_measurements);

    last_m = 0;
    for i = 1:num_poses
        filepath = ["data/meas-", num2str(i-1, '%05.f'), ".dat"];
        [m, landmark_ids, points] = loadMeasFile(filepath, num_landmarks);
        m += last_m;
        observations(:, last_m+1:m) = points(:,:);
        landmark_associations(1, last_m+1:m) = (i-1) * ones(1, m-last_m);
        landmark_associations(2, last_m+1:m) = landmark_ids(:,:);
        last_m = m;
    end

    last_m -= 1;
    observations = observations(:, 1:m);
    landmark_associations = landmark_associations(:, 1:m);

end

% Every measurement contains a sequence number
% groundtruth (of the robot) and odometry pose
% and measurement information:
%  point POINT_ID_CURRENT_MESUREMENT (1) ACTUAL_POINT_ID (2) IMAGE_POINT (3:4)
% The Image_Point represents the pair [col;row] where the landmark is observed in the image
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% m is a scalar containing the actual number of measurements in the file
%% landmark_ids is a 1 x m array containing the actual_point_id for each measurement
%% points is a 2 x m array containing [col_of_image_point; row_of_image_point]
function [m, landmark_ids, points] = loadMeasFile(filepath, num_landmarks)
    m = 1;
    landmark_ids = zeros(1, num_landmarks);
    points = zeros(2, num_landmarks);

    % Open the file
	fid = fopen(filepath, 'r');

    if fid == -1
        disp(['Error in reading file ', filepath]);
        return
    end

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
                landmark_ids(m) = p(1);
                points(:, m) = p(2:3);
                m += 1;
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

    m = m-1;
    landmark_ids = landmark_ids(:, 1:m);
    points = points(:, 1:m);
end
