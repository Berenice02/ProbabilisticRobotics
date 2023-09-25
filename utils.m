1;
% The following functions comes from the geometry_helpers_2d file, developed for
% the lectures of probabilistic robotics at Sapienza, University of Rome.
% Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti


function T=compute_odometry_trajectory(U)
	T=zeros(size(U,1),3);
	current_T=v2t(zeros(1,3));
	for i=1:size(U,1),
		u=U(i,1:3)';
		current_T*=v2t(u);
		T(i,1:3)=t2v(current_T)';
	end
end

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

#rotation matrix around x axis
function R=Rx(rot_x)
 c=cos(rot_x);
 s=sin(rot_x);
 R= [1  0  0;
     0  c  -s;
     0  s  c];
endfunction

#rotation matrix around y axis
function R=Ry(rot_y)
 c=cos(rot_y);
 s=sin(rot_y);
 R= [c  0  s;
     0  1  0;
     -s  0 c];
endfunction

#rotation matrix around z axis
function R=Rz(rot_z)
 c=cos(rot_z);
 s=sin(rot_z);
 R= [ c  -s  0;
      s  c  0;
      0  0  1];
endfunction

function R=angles2R(a)
  R=Rx(a(1))*Ry(a(2))*Rz(a(3));
endfunction;

#from 6d vector to homogeneous matrix
function T=v2t3d(v)
    T=eye(4);
    T(1:3,1:3)=angles2R(v(4:6));
    T(1:3,4)=v(1:3);
endfunction;

% Custom
#from 3d vector on 2d to homogeneous matrix on 3d
function T = from_2d_to_3d(v)
	v = [v(1:2)'; 0; 0; 0; v(3)];
	T = v2t3d(v);
end