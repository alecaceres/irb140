function tau=NEDiIn(robot, q, qd, qdd, grav, pext)
% tau=NEDiIn(robot, q,qd,qdd,grav,pext)
% robot: objeto robot
% q=vector 6x1 que representa el estado de las variables articulares
% qd=vector 6x1 que representa la velociad angular
% qdd=vector 6x1 que representa la aceleracion angular
% grav=vector 1x3  que representa la gravedad
% pext=vector de 3x2 que que representa la fuerza y par ejercidos sobre la
% herramienta
% tau=vector 1x6 que representa los pares ejercidos sobre las
% articulaciones

% NE1
alpha=robot.alpha;
a=robot.a;
d=robot.d;
offset=robot.offset;
% NE2
R=zeros(3,3,6);
for i=1:6
    aux=DH(q(i)+offset(i),d(i),a(i),alpha(i));
    R(:,:,i)=aux(1:3,1:3);
end

% NE3
w0=zeros(1,3)';
wd0=zeros(1,3)';
v0=zeros(1,3)';
vd0=-grav';
z0=[0 0 1]'; % aclarar!
p=zeros(3,6);
s=zeros(3,6);
I=zeros(3,3,6);
m=zeros(1,6);
L=robot.links;
for i=1:6
    p(:,i)=[a(i) d(i)*sin(q(i)+offset(i)) d(i)*cos(q(i)+offset(i))];
    s(:,i)=L(i).r;
    I(:,:,i)=L(i).I;
    m(i)=L(i).m;
end


% NE4
w=zeros(3,6);
for i=1:6
    if i==1
        w(:,i)=R(:,:,i)'*(w0+z0*qd(i));
    else
        w(:,i)=R(:,:,i)'*(w(:,i-1)+z0*qd(i));
    end
end
% NE5
wd=zeros(3,6);
for i=1:6
    if i==1
        wd(:,i)=R(:,:,i)'*(wd0+z0*qdd(i))+cross(w0,z0*qd(i));
    else
        wd(:,i)=R(:,:,i)'*(wd(:,i-1)+z0*qdd(i))+cross(w(:,i-1),z0*qd(i));
    end
end

% NE6
vd=zeros(3,6);
for i=1:6
    if i==1
        vd(:,i)=cross(wd(:,i),p(:,i))+cross(w(:,i),cross(w(:,i),p(:,i)))...
            +R(:,:,i)'*vd0;
    else
        vd(:,i)=cross(wd(:,i),p(:,i))+cross(w(:,i),cross(w(:,i),p(:,i)))...
            +R(:,:,i)'*vd(:,i-1);
    end
end

% NE7
ai=zeros(3,6);

for i=1:6
    ai(:,i)=cross(wd(:,i),s(:,i))+cross(w(:,i),cross(w(:,i),s(:,i)))+vd(:,i);
end

% NE8
f=zeros(3,6);
fext=pext(:,1);
for i=6:-1:1
    if i==6
        f(:,i)=eye(3,3)*fext+m(i)*ai(:,i);
    else
        f(:,i)=R(:,:,i+1)*f(:,i+1)+m(i)*ai(:,i);
    end
end
% NE9
n=zeros(3,6);
next=pext(:,2);
for i=6:-1:1
    if i==6
        n(:,i)=eye(3,3)*(next+cross(eye(3,3)*p(:,i),fext))+...
            cross((p(:,i)+s(:,i)),m(:,i)*ai(:,i))+...
            I(:,:,i)*wd(:,i)+cross(w(:,i),I(:,:,i)*w(:,i));
    else
        n(:,i)=R(:,:,i+1)*(n(:,i+1)+cross(R(:,:,i+1)*p(:,i),f(:,i+1)))+...
            cross((p(:,i)+s(:,i)),m(:,i)*ai(:,i))+...
            I(:,:,i)*wd(:,i)+cross(w(:,i),I(:,:,i)*w(:,i));
    end
end

% NE10
tau=zeros(1,6);
for i=6:-1:1
    tau(i)=n(:,i)'*R(:,:,i)*z0;
end

end
