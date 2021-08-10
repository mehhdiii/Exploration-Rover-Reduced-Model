function y_k = measurement_f(x_k, T)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
b = 1/2;
% var_w1 = 1e2;
% var_w2 = 1e2; 
% W_k = diag([var_w1 var_w2]); 
% 
% w = sqrt(W_k)*randn(2, 1); 
w = 0;
H = [1 0 b; 1 0 -b]; 
y_k = H*x_k + w;

end

