function evalvdot(P, Q, f_tilde)

M = inv(sqrtm(P));
syms pc1x pc1z pc1x_dot pc1z_dot theta1_dot theta1 theta12 theta12_dot ut tau ur;
m = 0;
count = 0;
R = 0.05; 
R_old = R;
while count < 5
    for i = 1:1:50000 % Number of random iterations
        y = (rand(size(P,1),1)-0.5);
        y = sqrt(R)*y/norm(y); % Random direction of a vector with sqrt(R) length
        z = M*y; % Random point on the level curve
        f_tilde_z = double(subs(f_tilde, {pc1x pc1z pc1x_dot pc1z_dot theta1 theta1_dot theta12 theta12_dot}, {z(1) z(2) z(3) z(4) z(5) z(6) z(7) z(8)}));

        % Lyapunov candidate derivative for the approximated system
        vdot = -z'*Q*z;
        
        % Lyapunov candidate derivative considering the approximated system
        % and the error between this and the real one
        vdottot = -z'*Q*z + 2*z'*P*f_tilde_z;

        if (vdottot > 0 && R_old == R) % && m == 0)
            disp("The point of the real system is external to the R.A.S.!")
            m = 1;
            vdottot
            z 
        end

        if vdot > 0 
            disp("The point of the approximated system is external to the R.A.S.!")
            break; 
        end
    end
    R = R*2;
    disp("R augmented")
    count = count + 1;
    if (m == 0)
        R_old = R;
    end
end
if m == 0
    disp(" The equilibrium points of the real system are G.A.S.!")
end
disp(" The equilibrium points of the approximated system are G.A.S.!")
