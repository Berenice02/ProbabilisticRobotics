source "./geometry_helpers.m"

% The following file contains some functions that will be useful to do computation
% Some of them comes from examples, developed for the lectures of probabilistic robotics
% at Sapienza, University of Rome. Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti

% applies a perturbation to the state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% XR:  is a 3x3xnum_poses array of homogeneous transform matrices
%% XL:  is a 3xnum_landmarks array of 3D positions
%% dx:  num_poses x 3 + num_landmarks x 3 perturbation vector
%%
%% XR:  XR after applying the perturbation
%% XL:  XL after applying the perturbation
function [XR, XL] = boxPlus(XR, XL, dx)
	global num_poses num_landmarks;
	% Robot poses
	for i=1:num_poses
		index_of_perturbation = getPoseIndex(i);
		dxr = dx(index_of_perturbation:index_of_perturbation+2);
		XR(:, :, i) = v2t(dxr) * XR(:, :, i);
	end
	% Landmark positions
	for i=1:num_landmarks
		index_of_perturbation = getLandmarkIndex(i);
		dxl = dx(index_of_perturbation:index_of_perturbation+2);
		XL(:, i) += dxl;
	end
end

% get the index of the perturbation vector corresponding to the index of the pose
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pose_index: the index of the pose for which we want the index
%%
%% idx: the index of the pose
function idx = getPoseIndex(pose_index)
    global num_poses num_landmarks;
    idx = ((pose_index-1) * 3) + 1;
end

% get the index of the perturbation vector corresponding to the index of the landmark
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% landmark_index: the index of the landmark for which we want the index
%%
%% idx: the index of the landmark
function idx = getLandmarkIndex(landmark_index)
    global num_poses num_landmarks;
    idx = (num_poses * 3) + ((landmark_index - 1) * 3) + 1;
end

% get error and Jacobian 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% XR:  is a 3x3 transform matrix representing the robot pose in world frame
%% XL:  is a 3x1 array representing the 3D position of the landmark in world frame
%% z:   is a 2x1 array representing the measured position of the landmark on the image
%%
%% e:   is a 2x1 array representing the error, aka the difference between prediction and measurement
%% Jr:  is a 2x3 derivative of the error wrt a perturbation of the pose
%% Jl:  is a 2x3 derivative of the error wrt a perturbation of the landmark
function [visible, e, Jr, Jl] = errorAndJacobian(xr, xl, z)
    global camera_infos;

	e = zeros(2,1);
	Jr = zeros(2,3);
	Jl = zeros(2,3);
	visible = false;

	[p_cam, visible] = point_in_camera(xl, xr);
	if not(visible)
		return;
	end
	
	[h, visible] = project_point(p_cam);
	if not(visible)
		return;
	end

	e = h-z;

	z_inv = 1/p_cam(3);
	z_inv_sq = z_inv * z_inv;
	J_proj = [z_inv	    0	    -p_cam(1) * z_inv_sq; 
              0	        z_inv	-p_cam(2) * z_inv_sq];

	w2c = world_to_camera(xr, include_k=false);
	R_t = w2c(1:3,1:3);

	J_icp = zeros(3);
  	J_icp(1:3, 1:2) = -R_t(:, 1:2);
    J_icp(1:3, 3) = R_t * [p_cam(2); -p_cam(1); 0];

	Jr = J_proj * camera_infos.K * J_icp;
	Jl = J_proj * camera_infos.K * R_t;
end

%%
function [chi_tot, XR, XL] = doLS(XR, XL, observations, landmark_associations)
	global num_poses num_landmarks;

	% size of the system
	system_size = 3 * num_poses + 3 * num_landmarks;
	H = zeros(system_size);
    b = zeros(system_size, 1);
	chi_tot = 0;
	
	for i = 1:length(observations)
		pose_id = landmark_associations(1, i);
		landmark_id = landmark_associations(2, i);

		z = observations(:, i);

		xr = XR(:, :, pose_id+1);
		xl = XL(:, landmark_id+1);

		[visible, e, Jr, Jl] = errorAndJacobian(xr, xl, z);

		if not(visible)
			continue;
		end

		chi = e'*e;
		chi_tot += chi;

		pose_index = getPoseIndex(pose_id+1);
		landmark_index = getLandmarkIndex(landmark_id+1);

		H(pose_index:pose_index+2, pose_index:pose_index+2) += Jr'*Jr;

		H(pose_index:pose_index+2, landmark_index:landmark_index+2) += Jr'*Jl;
		H(landmark_index:landmark_index+2, pose_index:pose_index+2) += Jl'*Jr;

		H(landmark_index:landmark_index+2,	landmark_index:landmark_index+2) += Jl'*Jl;

		b(pose_index:pose_index+2) += Jr'*e;
		b(landmark_index:landmark_index+2) += Jl'*e;
	end

	dx=zeros(system_size, 1);

	dx = -H\b;
	% dx(4:end) = -H(4:end, 4:end)\b(4:end, 1);
	[XR, XL] = boxPlus(XR, XL, dx);
end
