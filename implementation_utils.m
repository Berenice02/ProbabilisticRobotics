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

% get error and Jacobian of landmarks projections
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% XR:  is a 3x3 transform matrix representing the robot pose in world frame
%% XL:  is a 3x1 array representing the 3D position of the landmark in world frame
%% z:   is a 2x1 array representing the measured position of the landmark on the image
%%
%% e:   is a 2x1 array representing the error, aka the difference between prediction and measurement
%% Jr:  is a 2x3 derivative of the error wrt a perturbation of the pose
%% Jl:  is a 2x3 derivative of the error wrt a perturbation of the landmark
function [visible, e, Jr, Jl] = errorAndJacobianLandmarkProjections(xr, xl, z)
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

	w2c = world_to_camera(xr);
	R_t = w2c(1:3,1:3);

	J_icp = zeros(3);
  	J_icp(1:3, 1:2) = -R_t(:, 1:2);
	J_icp(1:3, 3) = R_t * [xl(2); -xl(1); 0];

	Jr = J_proj * camera_infos.K * J_icp;
	Jl = J_proj * camera_infos.K * R_t;
end

function [e, Ji, Jj]=poseErrorAndJacobianAA(xi, xj, z)
	e = zeros(6,1);
	Ji = zeros(6,3);
	Jj = zeros(6,3);

	g = inv(xi) * xj;
	e = flatten(g-z);

	Ri=xi(1:2,1:2);
	Rj=xj(1:2,1:2);

	% Jj(5:6, 1) = Ri' * [1; 0];
	% Jj(5:6, 2) = Ri' * [0; 1];

	% Rz0=[0 -1;
	%      1  0];
	% Jtheta = Ri' * Rz0 * Rj;
	% Jj(1:4, 3) = reshape(Jtheta, 4, 1);
	% Jj(5:6, 3) = Ri' * Rz0 * xj(1:2, 3);

	x1 = zeros(3,3);
	x1(1,3) = -1;
	x2 = zeros(3,3);
	x2(2,3) = -1;
	x3 = zeros(3,3);
	x3(1,2) = 1;
	x3(2,1) = -1;

	dx = flatten(inv(xi) * x1* xj);
	dy = flatten(inv(xi) * x2* xj);
	dt = flatten(inv(xi) * x3* xj);

	Jj = [dx dy dt];
	Ji=-Jj; 
end

function [e, Ji, Jj]=poseErrorAndJacobian(xi, xj, z)
	e = 0;
	Ji = zeros(1,3);
	Jj = zeros(1,3);

	g = inv(xi) * xj;
	h = norm(g(1:2, 3));
	z = norm(z(1:2, 3));

	e = h-z;

	Ri=xi(1:2,1:2);

	% Jj(1, 1) = norm(Ri' * [1; 0]);
	% Jj(1, 2) = norm(Ri' * [0; 1]);

	% Jj(1, 1) = 1;
	% Jj(1, 2) = 1;
	% Jj(1, 3) = [1 1] * xj(1:2, 3);

	% Jj(1,1) = g(1, 3);
	% Jj(1,2) = g(2, 3);

	% Jj(1,1) = Ri'(1,1)+ Ri'(2,1);
	% Jj(1,2) = Ri'(1,2)+ Ri'(2,2);
	% Jj(1,3) = [1 1] * Ri' * [- g(2, 3); g(1, 3)];

	Jj(1:2) = [1 1] * Ri';
	Jj(3) = xj(1:2, 3)' * Ri' * [1; -1];

	Ji=-Jj; 
end

%%
function [XR, XL, chi_tot, num_inliers, H, b] = doLS(XR, XL, observations, landmark_associations, initial_odometry, kernel_threshold_land, kernel_threshold_pose, damping)
	global num_poses num_landmarks;

	% size of the system
	system_size = 3 * num_poses + 3 * num_landmarks;
	H = zeros(system_size);
    b = zeros(system_size, 1);
	chi_tot = zeros(2, 1);
	num_inliers = zeros(2, 1);
	
	for i = 1:length(observations)
		pose_id = landmark_associations(1, i);
		landmark_id = landmark_associations(2, i);

		z = observations(:, i);

		xr = XR(:, :, pose_id+1);
		xl = XL(:, landmark_id+1);

		% pose-landmark constraint
		[visible, e, Jr, Jl] = errorAndJacobianLandmarkProjections(xr, xl, z);

		if not(visible)
			continue;
		end

		chi = e' * e;
		if (chi>kernel_threshold_land)
			e *= sqrt(kernel_threshold_land/chi);
			chi = kernel_threshold_land;
			chi_tot += kernel_threshold_land;
			continue;
		else
			num_inliers(1) += 1;
		end
		chi_tot(1) += chi;

		pose_index = getPoseIndex(pose_id+1);
		landmark_index = getLandmarkIndex(landmark_id+1);

		H(pose_index:pose_index+2, pose_index:pose_index+2) += Jr'*Jr;

		H(pose_index:pose_index+2, landmark_index:landmark_index+2) += Jr'*Jl;
		H(landmark_index:landmark_index+2, pose_index:pose_index+2) += Jl'*Jr;

		H(landmark_index:landmark_index+2,	landmark_index:landmark_index+2) += Jl'*Jl;

		b(pose_index:pose_index+2) += Jr'*e;
		b(landmark_index:landmark_index+2) += Jl'*e;
	end

	for i = 1:num_poses-1
		% pose-pose constraint
		xi = XR(:, :, i);
		xj = XR(:, :, i+1);
		z_ij = initial_odometry(:, :, i);
		[e, Ji, Jj] = poseErrorAndJacobian(xi, xj, z_ij);

		chi = e' * e;
		if (chi>kernel_threshold_pose)
			e *= sqrt(kernel_threshold_pose/chi);
			chi = kernel_threshold_pose;
		else
			num_inliers(2) += 1;
		end
		chi_tot(2) += chi;

		pose_i_index = getPoseIndex(i);
		pose_j_index = getPoseIndex(i+1);

		H(pose_i_index:pose_i_index+2, pose_i_index:pose_i_index+2) += Ji'*Ji;

		H(pose_i_index:pose_i_index+2, pose_j_index:pose_j_index+2) += Ji'*Jj;
		H(pose_j_index:pose_j_index+2, pose_i_index:pose_i_index+2) += Jj'*Ji;

		H(pose_j_index:pose_j_index+2,	pose_j_index:pose_j_index+2) += Jj'*Jj;

		b(pose_i_index:pose_i_index+2) += Ji'*e;
		b(pose_j_index:pose_j_index+2) += Jj'*e;

	end

	dx = zeros(system_size, 1);

	% dx = -H\b;
	dx(4:end) = -H(4:end, 4:end)\b(4:end, 1);
	[XR, XL] = boxPlus(XR, XL, dx);
end
