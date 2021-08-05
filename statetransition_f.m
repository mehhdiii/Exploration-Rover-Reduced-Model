function xk = statetransition_f(x_km1)
F = eye(3); 
T = 0.01; 
% var_v1 =1e-4; 
% var_v2 =1e-4; 
% var_v3 = 1e-4; 
% Q_k = diag([var_v1 var_v2 var_v3]); 
% v = sqrt(Q_k)*randn(3, 1);  
v = 0; 
xk = F*x_km1 + T*(v+[x_km1(2)*x_km1(3)]);
    
end

