

% states are  thata, thata_dot, x, x_dot

A = [-39.41     -4.882      0       0;
     -4.329     0.5029      0       0;
     0      0       -0.4072      -0.3614;
     0      0       0.4588        0.4071];
B = [26.28;
      6.802;
      0.002547;
     -1.199];
C = [11.08 0.09311 0 0;
     0 0 461 -0.252];
 
C = [11.08 0 0 0;
     0 0 461 0];
 
D = [0;
     0];

states = {'angle' 'angle_dot' 'x' 'x_dot' };
inputs = {'u'};
outputs = {'angle'; 'x' };

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

% poles = eig(A)
% 
% co = ctrb(sys_ss);
% controllability = rank(co)
% 
% Q = C'*C
% 
% Q = C'*C;
% R = 1;
% K = lqr(A,B,Q,R)
% 
% Ac = [(A-B*K)];
% Bc = [B];
% Cc = [C];
% Dc = [D];
% 
% states = {'angle' 'angle_dot' 'x' 'x_dot' };
% inputs = {'u'};
% outputs = {'angle'; 'x' };
% 
% sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% 
% 
% t = 0:0.01:10;
% r =0.2*ones(size(t));
% [y,t,x]=lsim(sys_cl,r,t);
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','robot angle (degree)robot position (m)')
% set(get(AX(2),'Ylabel'),'String','robot position (cm)')
% title('Step Response with LQR Control')

%////////////////////////////////////////

Ts = 1/200;

sys_d = c2d(sys_ss,Ts,'zoh')

co = ctrb(sys_d);
ob = obsv(sys_d);

controllability = rank(co)
observability = rank(ob)

A = sys_d.a;
B = sys_d.b;
C = sys_d.c;
D = sys_d.d;
Q = C'*C
Q=Q*0;

Q(1,1)=20;
Q(2,2)=0.1;
Q(3,3)=0;
Q(4,4)=0;
R = 1;
[K] = dlqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'angle' 'angle_dot' 'x' 'x_dot' };
inputs = {'u'};
outputs = {'angle'; 'x' };

sys_cl = ss(Ac,Bc,Cc,Dc,Ts,'statename',states,'inputname',inputs,'outputname',outputs);

figure(1)

X0=[0;0;0;0];

t = 0:0.005:10;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','robot angle (degree)')
set(get(AX(2),'Ylabel'),'String','robot position (cm)')
title('Step Response with Digital LQR Control')

%figure(2)
%step(sys_cl)