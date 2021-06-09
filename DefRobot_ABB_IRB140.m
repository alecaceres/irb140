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
      -180  50; % -230<=q<=50, pero se limita entre -180 y 180
      -180  180; % -200<=q<=200, pero se limita entre -180 y 180
      -115  115;
      -180  180]*pi/180; % -400<=q<=400, pero se limita entre -180 y 180

% Dinámica
M = 98; % kg, peso total del brazo excluyendo los cables al controlador
perc = 1/sum(0.3.^(0:5)); % aprox. 70% para que link_i sea 70% del link_i-1
r = -[
   0       0      d(1);
   0       a(2)   0;
   a(3)    0       0;
   0       0      d(4);
   0       0      0;
   0       0      0
]/2;
% creación de eslabones
for eje=1:6
    L(eje) = Revolute('d',d(eje),'a',a(eje),'alpha',alpha(eje),'offset',offset(eje), 'modified');
    L(eje).m = perc*M*0.3^(eje-1);
    L(eje).I = zeros(3,3);
    L(eje).r = r(eje,:);
end

robot = SerialLink(L);
robot.name='IRB_140';
robot.qlim=qlim;
%robot.d6=65e-3;