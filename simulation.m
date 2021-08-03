clear; close all; 
% define the system variables and model: 
T = 0.5;
F = eye(3); 

% define signal parameters: system
var_v1 =1e-6; 
var_v2 =1e-6; 
var_v3 = 1e-6; 
Q_k = diag([var_v1 var_v2 var_v3]); 

% define signal parameters: observation/sensor
var_w1 = 1e-4;
var_w2 = 1e-4; 
W_k = diag([var_w1 var_w2]); 
b = 0.5; %width of chassis
H = [1 0 b; 1 0 -b]; 

%simulation variables: 
ITER = 1e2; %number of iterations 

x_k = [1;0;0;]; % Plant initial state
xhat_last = [5; 0.01; 3]; %Last optimal predicted value (X_hat{k-1}): zero initially 
P_last = eye(3); % Last covarianceix value for the estimated states
y_last = 0; %last observation: zero initially 

%storage of values 
historyX_k = [0; 0; 0]; 
historyY_k = [0; 0];
historyX_predict = [0; 0; 0]; 

for k = 1:ITER   
    
    %generate noise values: 
    v = sqrt(Q_k)*randn(3, 1); 
    w = sqrt(W_k)*randn(2, 1); 
    
    %previous states:
    vx = x_k(1); 
    vy = x_k(2); 
    omega = x_k(3); 
    
    %generate xk and yk: 
    x_k = [vx; vy; omega] + T*v; 
    
    y_k = [vx+b*omega; vx-b*omega] + w;
    
    %run KF algorithm 
    [xhat_optimal, P_optimal] = KalmanFilter(y_k, xhat_last, P_last, F,Q_k,H,W_k); 
    xhat_last = xhat_optimal;
    y_last = y_k; 
    P_last = P_optimal; 
    historyX_k(:, k) = x_k; 
    historyY_k(:, k) = y_k; 
    historyX_predict(:, k) = xhat_optimal; 
end

%calculated trajectory from velocities: 
phi = atan2(historyX_predict(2, 1:end), historyX_predict(1, 1:end)); 
calc_trajectory = zeros(2, length(phi)); 

%true trajectory from original values: 
true_phi = atan2(historyX_k(2, 1:end), historyX_k(1, 1:end));
true_trajectory = zeros(2, length(true_phi)); 

for i=1:length(phi)-1
    %Predicted trajectory
    v = sqrt(historyX_predict(1, i)^2+historyX_predict(2, i)^2);
    calc_trajectory(1, i+1) = calc_trajectory(1, i) + v/historyX_predict(3, i) *(sin(phi(i)+T*historyX_predict(3, i)) - sin(phi(i)));
    calc_trajectory(2, i+1) = calc_trajectory(2, i) - v/historyX_predict(3, i) *(cos(phi(i)+T*historyX_predict(3, i)) - cos(phi(i)));
    
    %True trajectory
    true_v = sqrt(historyX_k(1, i)^2+historyX_k(2, i)^2); 
    true_trajectory(1, i+1) = true_trajectory(1, i) + true_v/historyX_k(3, i) *(sin(true_phi(i)+T*historyX_k(3, i)) - sin(true_phi(i)));
    true_trajectory(2, i+1) = true_trajectory(2, i) - true_v/historyX_k(3, i) *(cos(true_phi(i)+T*historyX_k(3, i)) - cos(true_phi(i)));
    
end

%figure plotting:
P = ITER-1; 
figure()

subplot 221
hold on
plot(historyX_k(1, end-P:end), 'kd:')
plot(historyX_predict(1, end-P:end), 'bs--', 'LineWidth',2)
xlabel("time", 'fontsize',12)
ylabel("x Velocity", 'fontsize',12)
title("X velocity of Linear system", 'fontsize',14)
lgd = legend('Stochastic Velocity',...
    'Estimated Velocity', 'location', 'best');lgd.FontSize = 14;
% lgd = legend('True Velocity', 'Predicted Velocity', 'location', 'best')
% lgd.FontSize = 12
hold off

subplot 222
hold on
plot(historyX_k(2, end-P:end), 'k-')
plot(historyX_predict(2, end-P:end), 'b--', 'LineWidth',2)
xlabel("time", 'fontsize',12)
ylabel("y Velocity", 'fontsize',12)
title("Y velocity of Linear system", 'fontsize',14)
lgd = legend('True Velocity', 'Predicted Velocity', 'location', 'best')
lgd.FontSize = 12
hold off

subplot 223
hold on
plot(historyX_k(3, end-P:end), 'k-')
plot(historyX_predict(3, end-P:end), 'b--', 'LineWidth',2)
xlabel("time", 'fontsize',12)
ylabel("angular Velocity", 'fontsize',12)
title("Angular velocity of Linear system", 'fontsize',14)
lgd = legend('True Velocity', 'Predicted Velocity', 'location', 'best')
lgd.FontSize = 12
hold off

subplot 224  
hold on 
plot(calc_trajectory(1, end-P:end),  calc_trajectory(2, end-P:end), 'k-')
plot(true_trajectory(1, end-P:end),  true_trajectory(2, end-P:end), 'b--', 'LineWidth',2)
xlabel("X", 'fontsize',12)
ylabel("Y", 'fontsize',12)
title("Trajectory of Linear system", 'fontsize',14)
lgd = legend('True Trajectory', 'Predicted Trajectory', 'location', 'best')
lgd.FontSize = 12
hold off

print -depsc results.eps