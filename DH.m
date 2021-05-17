% #-----------------------------------------------------------------------#
% #Definicion del objeto Robot para Robotics Toolbox Version 10.4
% #Modelo del robot: IRB 140
% #Marca del robot: ABB
% #Datos del Autor.
% #->Nombres: Alejandro David
% #->Apellidos: Cáceres Ortiz
% #->C.I: 5.612.050
% #->Correo: acaceres@fiuna.edu.py
% # Esta funcion recibe como argumentos los valores de los parametros DH
% correspondientes a una articulacion y devuelve la matriz de
% transformacion homogenea.
% #-----------------------------------------------------------------------#
function A=DH(theta, d, a, alpha)
A=eye(4);
for i=1:length(theta)
    Co=cos(theta(i));
    So=sin(theta(i));
    Ca=cos(alpha(i));
    Sa=sin(alpha(i));
    A=A*[Co       -So     0   a(i);
       So*Ca    Co*Ca   -Sa -d(i)*Sa;
       So*Sa    Co*Sa   Ca  d(i)*Ca;
       0        0       0       1];
end