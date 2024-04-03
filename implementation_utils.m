1;

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

