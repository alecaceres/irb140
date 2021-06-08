function q=CineI(robot,MTH,ci)
    % aún sin terminar
    a1= robot.a(2);
    a2= robot.a(3);
    d1= robot.d(1);
    d4= robot.d(4);
    d6= robot.d6;
    P6= [0 0 d6 1]';

    Ptcp= MTH*P6;
    Ptcp= num2cell(reshape(Ptcp,1,[]));
    [Pxtcp, Pytcp, Pztcp, ~]= deal(Ptcp{:});

    % Obtención de theta1
    theta1= atan2(Pytcp, Pxtcp);
    theta1_= pi+theta1;

    % Obtención de theta3
    s= Pztcp-d1;
    r=  sqrt((Pxtcp-a1*cos(theta1))^2+(Pytcp-a1*sin(theta1))^2);
    r_= -r;
    ra=  sqrt((Pxtcp-a1*cos(theta1_))^2+(Pytcp-a1*sin(theta1_))^2);
    ra_= -ra;
    cosalpha= (s^2+r^2-a2^2-(d4+d6)^2)/(2*a2*(d4+d6));
    cosalphaa= (s^2+ra^2-a2^2-(d4+d6)^2)/(2*a2*(d4+d6));
    sinalpha= sqrt(1-cosalpha^2);
    sinalpha_= -sinalpha;
    sinalphaa= sqrt(1-cosalphaa^2);
    sinalphaa_= -sinalphaa;
    alpha= atan2(sinalpha, cosalpha);
    alpha_= atan2(sinalpha_, cosalpha);
    alphaa= atan2(sinalphaa, cosalphaa);
    alphaa_= atan2(sinalphaa_, cosalphaa);

    theta3= -(pi/2+alpha);
    theta3_= -(pi/2+alpha_);
    theta3a= -(pi/2+alphaa);
    theta3a_= -(pi/2+alphaa_);

    % Obtención de theta2
    omega= atan2(s, r);
    omega_= atan2(s, r_);
    omegaa= atan2(s, ra);
    omegaa_= atan2(s, ra_);
    lambda= atan2((d4+d6)*sinalpha, a2+(d4+d6)*cosalpha);
    lambda_= atan2((d4+d6)*sinalpha_, a2+(d4+d6)*cosalpha);
    lambdaa= atan2((d4+d6)*sinalphaa, a2+(d4+d6)*cosalphaa);
    lambdaa_= atan2((d4+d6)*sinalphaa_, a2+(d4+d6)*cosalphaa);
    theta2= -(omega-lambda-pi/2);
    theta2_= -(omega_-lambda-pi/2);
    theta2a= -(omega-lambda_-pi/2);
    theta2a_= -(omega_-lambda_-pi/2);
    theta2b= -(omegaa-lambdaa-pi/2);
    theta2b_= -(omegaa_-lambdaa-pi/2);
    theta2c= -(omegaa-lambdaa_-pi/2);
    theta2c_= -(omegaa_-lambdaa_-pi/2);

    % Solución para el punto de muñeca
    theta123 = [theta1   theta2   theta3;
                theta1   theta2_  theta3;
                theta1   theta2a  theta3_;
                theta1   theta2a_ theta3_;
                theta1_  theta2b  theta3a;
                theta1_  theta2b_ theta3a;
                theta1_  theta2c  theta3a_;
                theta1_  theta2c_ theta3a_;];

    % Obtener soluciones completas
    for i=1:len(theta123)
        R06=rotz(x)*roty(y)*rotz(z);
        R03= MTH(1:3,1:3);
        R36= R03'*R06;
        R36cells= num2cell(reshape(R36,1,[]));
        [~, ~, g13, ~, ~, g23, g31, g32, g33]= deal(R36cells{:});

        theta5= atan2(sqrt(g31^2+g32^2), g33);
        theta4= atan2(g32/sin(theta5), -g31/sin(theta5));
        theta6= atan2(g23/sin(theta5), g13/sin(theta5));
        theta5_= -theta5;
        theta4_= pi+theta4;
        theta6_= pi+theta6;

    end

end