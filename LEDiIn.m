function tau=LEDiIn(robot, q, qd, qdd, g, pext)

    N=3; % se toman los primeros N grados de libertad

    % LE1
    alpha=robot.alpha;
    a=robot.a;
    d=robot.d;
    offset=robot.offset;

    % LE2

    % LE3
    U=zeros(4,4,N,N);
    Q = [0 -1  0   0;
          1 0   0   0;
          0 0   0   0;
          0 0   0   0]; % ya que todos los eslabones son rotatorios
    for i=1:N
        for j=1:i
            A0j_1=DH(offset(1:j-1),d(1:j-1),a(1:j-1),alpha(1:j-1));
            Aj_1i=DH(offset(j-1:i),d(j-1:i),a(j-1:i),alpha(j-1:i));
            U(i,j)=A0j_1*Q*Aj_1i;
            end
        end
    end

    % LE4
    Ud = zeros(4,4,N,N,N);
    for i=1:N
        for j=1:i
            for k=1:i
                if k>j
                    A0j_1=DH(offset(1:j-1),d(1:j-1),a(1:j-1),alpha(1:j-1));
                    Aj_1k_1=DH(offset(j-1:k-1),d(j-1:k-1),a(j-1:k-1),alpha(j-1:k-1));
                    Ak_1i=DH(offset(k-1:i),d(k-1:i),a(k-1:i),alpha(k-1:i));
                    Ud(i,j,k)=A0j_1*Q*Aj_1k_1*Q*Ak_1i;
                else
                    A0k_1=DH(offset(1:k-1),d(1:k-1),a(1:k-1),alpha(1:k-1));
                    Ak_1j_1=DH(offset(k-1:j-1),d(k-1:j-1),a(k-1:j-1),alpha(k-1:j-1));
                    Aj_1i=DH(offset(j-1:i),d(j-1:i),a(j-1:i),alpha(j-1:i));
                    Ud(i,j,k)=A0j_1*Q*Aj_1k_1*Q*Ak_1i;
            end
        end
    end

    % LE5
    J=zeros(4,4,N);
    for elemento=1:N
        r = [robot.links(elemento).r 1];
        for i=1:N
            for j=1:N
                J(elemento,i,j)=r(i)*r(j);
            end
        end
        J(elemento)=J(elemento)*robot.links(element).m;
    end

    % LE6
    D=zeros(N,N);
    for i=1:N
        for j=1:N
            k=max(i,j);
            D(i,j)=sum(diag(U(k,j)*J(k)*U(k,j)'));
        end
    end

    % LE7
    Hikm=zeros(N,N,N);
    for i=1:N
        for k=1:N
            for m=1:N
                j=max(i,j,m);
                Hikm(i,k,m)=sum(diag(Ud(j,k,m)*J(j)*U(j,i)'));
            end
        end
    end

    % LE8
    H=zeros(N)';
    for i=1:N
        for k=1:N
            for m=1:N
                H(i)=Hikm(i,k,m)*qd(k)*qd(m);
            end
        end
    end

    % LE9
    C=zeros(N)';
    for i=1:N
        for j=1:N
            C(i)=-robot.links(j).m*g*U(i,j)*robot.links(j).r;
        end
    end

    % LE10
    tau=D*qdd+H+C;
end