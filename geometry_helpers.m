1;
% The following functions comes from the geometry_helpers_2d file, developed for
% the lectures of probabilistic robotics at Sapienza, University of Rome.
% Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti

% computes the pose 2d pose vector v from an homogeneous transform A
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function v=t2v(A)
	v(1:2, 1)=A(1:2,3);
	v(3,1)=atan2(A(2,1),A(1,1));
end

% computes the homogeneous transform matrix A of the pose vector v
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function A=v2t(v)
  	c=cos(v(3));
  	s=sin(v(3));
	A=[c, -s, v(1) ;
	s,  c, v(2) ;
	0   0  1  ];
end

% normalizes and angle between -pi and pi
% th: input angle
% o: output angle
function o = normalizeAngle(th)
	o = atan2(sin(th),cos(th));
end

#computes the derivative of atan2(p.y/p.x);
function J = J_atan2(p)
  n2=p'*p;
  J= 1./n2 * [-p(2) p(1)];
endfunction

% The following functions comes from the geometry_helpers_3d file, developed for
% the lectures of probabilistic robotics at Sapienza, University of Rome.
% Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti

#rotation matrix around z axis
function R=Rz(rot_z)
 c=cos(rot_z);
 s=sin(rot_z);
 R= [ c  -s  0;
      s  c  0;
      0  0  1];
endfunction

% Custom
#from 3d vector on 2d to homogeneous matrix on 3d
function T = from_3dv_to_3dt(v)
	T = eye(4);
	T(1:3,1:3) = Rz(v(3));
	T(1:3,4)=[v(1:2)'; 0];
end

#from homogeneous matrix on 2d to homogeneous matrix on 3d
function T = from_2dt_to_3dt(T2)
	T = eye(4);
	T(1:2,1:2) = T2(1:2,1:2);
	T(1:2,4) = T2(1:2,3);
end

#from homogeneous matrix on 3d to homogeneous matrix on 2d
function T = from_3dt_to_2dt(T3)
	T = eye(3);
	T(1:2,1:2) = T3(1:2,1:2);
	T(1:2,3) = T3(1:2,4);
end

function flattened = flatten(X)
	flattened = zeros(6, 1);
	flattened(1:2) = X(1:2, 1);
	flattened(3:4) = X(1:2, 2);
	flattened(5:6) = X(1:2, 3);
end