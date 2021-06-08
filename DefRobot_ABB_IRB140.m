% #-----------------------------------------------------------------------#
% #Definicion del objeto Robot para Robotics Toolbox Version 10.4
% #Modelo del robot: IRB 140
% #Marca del robot: ABB
% #Datos del Autor.
% #->Nombres: Alejandro David
% #->Apellidos: Cáceres Ortiz
% #->C.I: 5.612.050
% #->Correo: acaceres@fiuna.edu.py
% #-----------------------------------------------------------------------#

close all
clear


% Parámetros DH
d=[352  0   0   380 0   0]/1000;
a=[0    70  360 0   0   0]/1000;
alpha=[0    -90 0   -90 90  -90]*pi/180;
offset=[0 -90 0 0 0 0]*pi/180;
qlim=[-180  180;
      -90   110;
      -230  50;
      -200  200;
      -115  115;
      -400  400]*pi/180;

% creación de eslabones
for eje=1:6
    L(eje) = Revolute('d',d(eje),'a',a(eje),'alpha',alpha(eje),'offset',offset(eje), 'modified');
end

robot = SerialLink(L);
robot.name='IRB_140';
robot.qlim=qlim;
robot.d6=65e-3;