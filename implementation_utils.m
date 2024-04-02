1;

% The following file contains some functions that will be useful to do computation
% Some of them comes from examples, developed for the lectures of probabilistic robotics
% at Sapienza, University of Rome. Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti

% applies a perturbation to the state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% XR:  is a 3x3xnum_poses array of homogeneous transform matrices
%% XL:  is a 3xnum_landmarks array of 3D positions
%% dx:  the perturbation vector of appropriate dimensions
%%
%% XR:  XR after applying the perturbation
%% XL:  XL after applying the perturbation
function [XR, XL]=boxPlus(XR, XL, dx)
  global num_poses, num_landmarks;

end

% transform a long perturbation vector into two matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% dx: num_poses x 3 + num_landmarks x 3 perturbation vector
%%
%% xr: 3xnum_poses matrix of robot poses perturbations
%% xl: 3xnum_landmarks matrix of landmark positions perturbations
function [xr, xl]= v2mPerturbation(dx)
    global num_poses, num_landmarks;
    xr = zeros(3, num_poses);
    xl = zeros(3, num_landmarks);

    

end

% transform two perturbation matrices into a long vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% xr: 3xnum_poses matrix of robot poses perturbations
%% xl: 3xnum_landmarks matrix of landmark positions perturbations
%%
%% dx: num_poses x 3 + num_landmarks x 3 perturbation vector
function dx= m2vPerturbation(xr, xl)
    global num_poses, num_landmarks;
    dx = zeros(num_poses*3+num_landmarks*3, 1);
    for i=1:num_poses
        dx(i) = xr(:, i)';
    end

    for i=1:num_landmarks
        dx(i+num_poses*3) = xl(:, i)';
    end

end