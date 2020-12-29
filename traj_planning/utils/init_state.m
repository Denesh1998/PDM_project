function [ s ] = init_state( start, yaw )
%INIT_STATE Initialize 13 x 1 state vector

s     = zeros(12,1);
phi0   = 0.0;
theta0 = 0.0;
psi0   = yaw;
Rot0   = RPYtoRot_ZXY(phi0, theta0, psi0);
Quat0  = RotToQuat(Rot0);
s(1)  = start(1); %x
s(2)  = start(2); %y
s(3)  = start(3); %z
s(4)  = 0;        %xdot
s(5)  = 0;        %ydot
s(6)  = 0;        %zdot
s(7)  = phi0; %qw
s(8)  = theta0; %qx
s(9)  = psi0; %qy
% s(10) = Quat0(4); %qz
s(10) = 0;        %p
s(11) = 0;        %q
s(12) = 0;        %r

end