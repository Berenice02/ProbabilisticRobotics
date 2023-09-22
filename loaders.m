% The following functions are based on the loadG2o function, developed for
% the lectures of probabilistic robotics at Sapienza, University of Rome.
% Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti

function [world_landmark, camera_infos, traj, gt_poses, odom_poses, observations] = loaders()
    world_landmark = loadWorld();
    camera_infos = loadCameraInfos();
    traj = loadTrajectory();
    [gt_poses, odom_poses, observations] = loadMeasurements();
end

% world.dat contains information about the map
% Every row contains: LANDMARK_ID (1) POSITION (2:4)
function world_landmark = loadWorld()
    W=load("data/world.dat");
    world_landmark.id = W(:, 1);
    world_landmark.position = W(:, 2:4);
end


% camera.dat contains information about the camera used to gather data:
% camera matrix
% cam_transform: pose of the camera w.r.t. robot
% z_near/z_far how close/far the camera can perceive stuff
% width/height of images
function camera_infos = loadCameraInfos()
    % Open the file
	fid = fopen("data/camera.dat", 'r');

    while true
		% Get current line
		c_line = fgetl(fid);

		% Stop if EOF
		if c_line == -1
			break;
		end

		% Remove leading and trailing whitespace
        % And split the line using space as separator
        c_line = strtrim(c_line);
		elements = strsplit(c_line,' ');

        switch(elements{1})
            case 'camera'
                if strcmp (elements{2}, 'matrix:')
                    camera_infos.cm = extractMatrices(fid, 3);
                end
            case 'cam_transform:'
                camera_infos.cam_transform = extractMatrices(fid, 4);
            case 'z_near:'
                camera_infos.z_near = elements{2};
            case 'z_far:'
                camera_infos.z_far = elements{2};
            case 'width:'
                camera_infos.width = elements{2};
            case 'height:'
                camera_infos.height = elements{2};
            otherwise
                if exist(elements{1})
                    disp(['Error in reading element ', elements{1}, ...
                    'while reading file ', filepath]);
                end
                continue;
        end
    end
end

function out = extractMatrices(fid, lines)
    for j = 1:lines
        c_line = strtrim(fgetl(fid));
        elements = strsplit(c_line,' ');
        for k = 1:length(elements)
            out(j, k) = str2double(elements{k});
        end
    end
end


% trajectory.dat POSE_ID (1) ODOMETRY_POSE (2:4) GROUNDTRUTH_POSE (5:7)
% The ODOMETRY_POSE is obtained by adding Gaussian Noise (0; 0.001) to the actual robot commands
function traj = loadTrajectory()
    # TODO
    # Those are the same as the ones from measurement files
    T=load("data/trajectoy.dat");
    % traj.pose_id = T(:, 1);
    % traj.odom_poses = T(:, 2:4);
    % traj.gt_poses = T(:, 5:7);
    traj = T;
end


% meas-XXXXX.dat contains information about the XXXXX measurement
function [gt_poses, odom_poses, observations] = loadMeasurements()
    for i = 1:200
        filepath = ["data/meas-00", num2str(i-1, '%03.f'), ".dat"];
        [seq, gt_pose, odom_pose, points] = loadMeasFile(filepath);
        gt_poses(seq+1, :) = gt_pose;
        odom_poses(seq+1, :) = odom_pose;
        observations(seq+1, 1).obs = points(:,:);
    end
end

% Every measurement contains a sequence number, groundtruth (of the robot) and odometry pose and measurement information:
% - point POINT_ID_CURRENT_MESUREMENT (1) ACTUAL_POINT_ID (2) IMAGE_POINT (3:4)
% The Image_Point represents the pair [col;row] where the landmark is observed in the image
function [seq, gt_pose, odom_pose, points] = loadMeasFile(filepath)
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
            case 'seq:'
                seq = str2num(elements{2});
            case 'gt_pose:'
                gt_pose = extractPose(elements);
            case 'odom_pose:'
                odom_pose = extractPose(elements);
            case 'point'
                p = extractPoint(elements);
                points = [points; p];
            otherwise
                if exist(elements{1})
                    disp(['Error in reading element ', elements{1}, ...
                    'while reading file ', filepath]);
                end
                continue;
        end
    end
end

function out = extractPose(elements)
    x_pose = str2double(elements{2});
    y_pose = str2double(elements{3});
    th_pose = str2double(elements{4});
    out = [x_pose, y_pose, th_pose];
end

function out = extractPoint(elements)
    meas_id = str2double(elements{2});
    gt_id = str2double(elements{3});
    x_p = str2double(elements{4});
    y_p = str2double(elements{5});
    # POINT_ID_CURRENT_MESUREMENT
    out.meas_id = meas_id;
    # ACTUAL_POINT_ID
    out.gt_id = gt_id;
    # IMAGE_POINT
    out.image_point = [x_p, y_p];
end
