clear; close all; 
ITER = 1e2; 



%covariances: 
var_v1 =1e-4; 
var_v2 =1e-4; 
var_v3 = 1e-4; 
var_w1 = 1e-2;
var_w2 = 1e-2; 
Q_k = diag([var_v1 var_v2 var_v3]); 
W_k = diag([var_w1 var_w2]); 


%system 
F = eye(3);
T = 0.01; 
b = 1/2; %width of chassis
H = [1 0 b; 1 0 -b]; 

%generate values 
x_k = [0; 0; 0];
history = zeros(3+3, 1); 
filter = trackingEKF('StateTransitionFcn', @statetransition_f, 'MeasurementFcn', @measurement_f,...
    'State',x_k, 'StateTransitionJacobianFcn', @statetransition_j, 'MeasurementJacobianFcn', @measurement_j,...
    'ProcessNoise', Q_k, 'MeasurementNoise', W_k)

for k = 1:ITER
    
    
    %generate noise values: 
    v = sqrt(Q_k)*randn(3, 1); 
    w = sqrt(W_k)*randn(2, 1); 
        
    %generate xk and yk: 
    x_k = F*x_k + T*(v+[x_k(2)*x_k(3);-x_k(3)*x_k(3);0]);     
    y_k = H*x_k + w;
    [xpred, Ppred] = predict(filter);
    [xcorr, Pcorr] = correct(filter, y_k);
    history(:, k) = vertcat(x_k, xcorr); 
   
    
    
end
figure()
hold on 

plot(history(4, :), 'linewidth', 2)
plot(history(1, :), 'k-')
title("x velocity ")
legend("Filtered output", "True Output")
hold off
figure()
hold on 
plot(history(5, :), 'linewidth', 2)
plot(history(2, :), '-')
title("y velocity ")
legend("Filtered output", "True Output")
hold off