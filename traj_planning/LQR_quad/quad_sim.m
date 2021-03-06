
%%LQR for euler angles
Ix = 1.43e-5;
Iy = 1.43e-5;
Iz = 2.89e-5;

A = [0      0               0           1  0   0  ;
     0      0               0           0  1   0  ;
     0      0               0           0  0   1  ;
     0      0               0           0  0   0 ;
     0      0               0           0  0   0 ;
     0      0               0           0  0   0 ];

B = [ 0 0 0;
      0 0 0; 
      0 0 0;
      1/Ix 0 0;
      0  1/Iy 0;
      0  0 1/Iz]; 
 
 
C = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];
D = [ 0 0 0; 
       0 0 0;
       0 0 0]; 

states = { 'phi' 'theta' 'psi' 'p' 'q' 'r'};
inputs = { 'tau_phi' 'tau_theta' 'tau_psi'};
outputs = {'phi'; 'theta' ; 'psi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

poles = eig(A)
co = ctrb(sys_ss);
controllability = rank(co)

Q = (10^5)*[100 0 0 0 0 0;
     0  1000 0 0 0 0 ;
     0 0  100  0 0 0;
     0 0  0  10 0 0;
     0 0 0 0  100 0;
     0 0 0 0 0 10];
R = [1 0 0;
    0  1 0;
    0 0  1]
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];
poles = eig(A-B*K)

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
A_ref = [ones(size(t)) ; 0*ones(size(t)) ; 0*ones(size(t)) ]';

[y,t,x]=lsim(sys_cl,A_ref,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')


K1 = [3000 0 0 300 0 0;
    0 3000 0 0 300 0;
    0 0 3000 0 0 300];
poles_1 = eig(A-B*K1)
% 
% %%LQR for x,y
% m = 0.03;
% A = [0      0               1              0  ;
%      0      0               0              1  ;
%      0      0               0              0  ;
%      0      0               0              0]  
% B = [ 0  ;
%       0  ; 
%       1/m ;
%       1/m ]; 
%  
%  
% C = [1 0 0 0; 
%     0 1 0 0];
% D = [ 0 ; 
%       0]; 
% 
% states = { 'x' 'y' 'xd' 'yd'};
% inputs = { 'f'};
% outputs = {'x'; 'y' };
% 
% sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
% 
% poles = eig(A)
% co = ctrb(sys_ss);
% controllability = rank(co)
% Q = C'*C
% Q = C'*C;
% R = eye(1);
% K = lqr(A,B,Q,R)
% 
% Ac = [(A-B*K)];
% Bc = [B];
% Cc = [C];
% Dc = [D];
% poles = eig(A-B*K)
% 
% sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% 
% t = 0:0.01:1;
% A_ref = [0.1*ones(size(t)) ; 0.1*ones(size(t)) ; 0*ones(size(t)) ]';
% 
% [y,t,x]=lsim(sys_cl,A_ref,t);
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','cart position (m)')
% set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
% title('Step Response with LQR Control')
% 
% 
% 
% % Q = C'*C;
% % Q(1,1) = 5000;
% % Q(3,3) = 100
% % R = 1;
% % K = lqr(A,B,Q,R)
% % 
% % Ac = [(A-B*K)];
% % Bc = [B];
% % Cc = [C];
% % Dc = [D];
% % 
% % states = {'x' 'x_dot' 'phi' 'phi_dot'};
% % inputs = {'r'};
% % outputs = {'x'; 'phi'};
% % 
% % sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% % 
% % t = 0:0.01:5;
% % r =0.2*ones(size(t));
% % [y,t,x]=lsim(sys_cl,r,t);
% % [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% % set(get(AX(1),'Ylabel'),'String','cart position (m)')
% % set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
% % title('Step Response with LQR Control')