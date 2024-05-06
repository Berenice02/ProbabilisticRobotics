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
function [XR, XL]=boxPlus(XR, XL, dx)
	global num_poses, num_landmarks;
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
		XL(:, :, i) += dxl;
	end
end

% get the index of the perturbation vector corresponding to the index of the pose
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pose_index: the index of the pose for which we want the index
%%
%% idx: the index of the pose
function idx = getPoseIndex(pose_index)
    global num_poses, num_landmarks;
    idx = ((pose_index-1) * 3) + 1;
end

% get the index of the perturbation vector corresponding to the index of the landmark
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% landmark_index: the index of the landmark for which we want the index
%%
%% idx: the index of the landmark
function idx = getLandmarkIndex(landmark_index)
    global num_poses, num_landmarks;
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
function [visible, e, Jr, Jl] = errorAndJacobian(XR, XL, z)
    global camera_infos;

	e = zeros(2,1);
	Jr = zeros(2,3);
	Jl = zeros(2,3);
	visible = false;

	[p_cam, visible] = point_in_camera(XL, XR);
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

	XR_4 = from_2dt_to_3dt(XR);
	w2c = XR_4*camera_infos.T;
	R_t = w2c(1:3,1:3)';

	J_icp = zeros(3);
  	J_icp(1:3, 1:2) = -R_t(:, 1:2);
    J_icp(1:3, 3) = R_t * [p_cam(2); -p_cam(1); 0];

	Jr = J_proj * camera_infos.K * J_icp;
	Jl = J_proj * camera_infos.K * R_t;
end
