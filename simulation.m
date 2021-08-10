clear; close all; 
ITER = 1e2; 



%covariances: 
var_v1 =1e-4; 
var_v2 =1e-4; 
var_v3 = 1e-4; 
var_w1 = 1e-4;
var_w2 = 1e-4; 
Q_k = diag([var_v1 var_v2 var_v3]); 
W_k = diag([var_w1 var_w2]); 


%system 
F = eye(3);
T = 0.01; 
b = 1/2; %width of chassis
H = [1 0 b; 1 0 -b]; 

%storage of values 
position = zeros(3, 1); 

%required states:
x_required = zeros(3, 1); %[px; py; phi]
x_ref = zeros(3,1); 

%generate values 
x_k = [0; 0; 0];
history = zeros(2+3, 1); 

filter = ExtendedKF(@statetransition_f, @measurement_f,...
     @statetransition_j, @measurement_j,...
    Q_k, W_k, T, x_k)
% 
for k = 1:ITER

    [Xpred, Ppred] = filter.predict(); 
    [Xcorr, Pcorr] = filter.correct(); 
    

end

 
% figure()
subplot 311
hold on 
plot(filter.Xtrue(1, :), 'red','linewidth', 2)
plot(filter.Xhistory(1, :), 'b--', 'linewidth', 1)
title("x velocity ")
legend("True Output", "Filtered output")
hold off

% figure()
subplot 312
hold on 
plot(filter.Xtrue(2, :), 'red','linewidth', 2)
plot(filter.Xhistory(2, :), 'b--', 'linewidth', 1)
title("y velocity ")
legend("True Output", "Filtered output")
hold off

% figure()
subplot 313
hold on 
plot(filter.Xtrue(3, :), 'red','linewidth', 2)
plot(filter.Xhistory(3, :), 'b--', 'linewidth', 1)
title("Angular velocity ")
legend("True Output", "Filtered output")
hold off