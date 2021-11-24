function y_k = measurement_f(x_k, T, w)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
b = 1/2;

H = [1 0 b; 1 0 -b]; 
y_k = H*x_k + w;

end

