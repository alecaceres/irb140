close all
clear

syms d2 m1 m2 L1
% Parámetros DH
d=[0 d2];
a=[0 0];
alpha=[-90 0]*pi/180;
offset=[0 0]*pi/180;

% Dinámica
M = [m1 m2];
r = [
   0       0      L1;
   0       0      0
];
% creación de eslabones
L1=Link('d',d(1),'a',a(1),'alpha',alpha(1),'offset',offset(1));
L2=Link('d',d(2),'a',a(2),'alpha',alpha(2),'offset',offset(2));
L1.m=m1;
L2.m=m2;
L1.r=r(1,:);
L2.r=r(2,:);

robot = SerialLink([L1 L2]);
robot.name='Test';