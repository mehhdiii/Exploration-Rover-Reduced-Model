function xk = state_transition(x_km1)
F = eye(3); 
T = 0.01; 
v = 0; 
xk = F*x_km1 + T*(v+[x_km1(2)*x_km1(3)]); 
    
end

