function jac_x = state_j(x_km1)

vx = x_km1(1);
vy = x_km1(2);
omega = x_km1(3);
T = 0.01
jac_x = [1, T*omega,  T*vy; 
        -T*omega,       1, -T*vx; 
         0,       0,     1]; 
end

