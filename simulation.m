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

%states:
x_k = [1;0;0;]; % Plant initial state
xhat_last = [5; 0.01; 3]; %Last optimal predicted value (X_hat{k-1}): zero initially 
P_last = eye(3); % Last covarianceix value for the estimated states
y_last = 0; %last observation: zero initially 

%required states:
x_required = zeros(3, 1); %[px; py; phi]
x_ref = zeros(3,1); 

%storage of values 
historyX_k = [0; 0; 0]; 
historyX_required =zeros(3, ITER);
historyY_k = [0; 0];
historyX_predict = [0; 0; 0]; 


for k = 1:ITER   
    
    %generate noise values: 
    v = sqrt(Q_k)*randn(3, 1); 
    w = sqrt(W_k)*randn(2, 1); 
        
    %generate xk and yk: 
    x_k = F*x_k + T*v; 
    
    y_k = H*x_k + w;
    
    %run KF algorithm 
    [xhat_optimal, P_optimal] = KalmanFilter(y_k, xhat_last, P_last, F,Q_k,H,W_k); 
    
    %calculate required states: [px, py, phi] 
    
    %xhat_optimal = [vx, vy, wn]
    x_required(3) = x_required(3) + T*xhat_optimal(3); %calculate phi
    Rf = [cos(x_required(3)) -sin(x_required(3)) 0; sin(x_required(3)) cos(x_required(3)) 0; 0 0 1]; %define rotation matrix:
    uc = Rf*xhat_optimal; %rotate the velocities 
    x_required(1:2) = [x_required(1) + uc(1)/uc(3)*(sin(x_required(3)+T*xhat_optimal(3)) - sin(x_required(3)));
                       x_required(2) - uc(2)/uc(3)*(cos(x_required(3)+T*xhat_optimal(3)) - cos(x_required(3)));
                        ]; 
    %required states: REFERENCE values:
    x_ref(3) = x_ref(3) + T*x_k(3); %calculate phi
    Rf = [cos(x_ref(3)) -sin(x_ref(3)) 0; sin(x_ref(3)) cos(x_ref(3)) 0; 0 0 1]; %define rotation matrix:
    uc = Rf*x_k; %rotate the velocities 
    x_ref(1:2) = [x_ref(1) + uc(1)/uc(3)*(sin(x_ref(3)+T*x_k(3)) - sin(x_ref(3)));
                       x_ref(2) - uc(2)/uc(3)*(cos(x_ref(3)+T*x_k(3)) - cos(x_ref(3)));
                        ]; 
    
                    
    %update values for next iteration:
    xhat_last = xhat_optimal;
    y_last = y_k; 
    P_last = P_optimal; 
    
    %Store values
    historyX_k(:, k) = x_k; 
    historyX_required(:, k) = x_required;
    historyX_ref(:, k) = x_ref;
    historyY_k(:, k) = y_k; 
    historyX_predict(:, k) = xhat_optimal; 
end

% %calculated trajectory from velocities: 
% phi = atan2(historyX_predict(2, 1:end), historyX_predict(1, 1:end)); 
% calc_trajectory = zeros(2, length(phi)); 
% 
% %true trajectory from original values: 
% true_phi = atan2(historyX_k(2, 1:end), historyX_k(1, 1:end));
% true_trajectory = zeros(2, length(true_phi)); 

% for i=1:length(phi)-1
%     %Predicted trajectory
%     Vg = [cos(phi(i)) -sin(phi(i)); sin(phi(i)) cos(phi(i)) ] * historyX_predict(1:2, i);  
%     v = sqrt(Vg(1)^2+Vg(2)^2);
% %     calc_trajectory(1, i+1) = calc_trajectory(1, i) + v/historyX_predict(3, i) *(sin(phi(i)+T*historyX_predict(3, i)) - sin(phi(i)));
% %     calc_trajectory(2, i+1) = calc_trajectory(2, i) - v/historyX_predict(3, i) *(cos(phi(i)+T*historyX_predict(3, i)) - cos(phi(i)));
%     calc_trajectory(1, i+1) = calc_trajectory(1, i) + v/historyX_predict(3, i) *(sin(phi(i)+T*historyX_predict(3, i)) - sin(phi(i)));
%     calc_trajectory(2, i+1) = calc_trajectory(2, i) - v/historyX_predict(3, i) *(cos(phi(i)+T*historyX_predict(3, i)) - cos(phi(i)));
%     %True trajectory
%     Vg = [cos(phi(i)) -sin(phi(i)); sin(phi(i)) cos(phi(i)) ] * historyX_k(1:2, i);
%     v = sqrt(Vg(1)^2+Vg(2)^2);
%     true_trajectory(1, i+1) = true_trajectory(1, i) + v/historyX_k(3, i) *(sin(true_phi(i)+T*historyX_k(3, i)) - sin(true_phi(i)));
%     true_trajectory(2, i+1) = true_trajectory(2, i) - v/historyX_k(3, i) *(cos(true_phi(i)+T*historyX_k(3, i)) - cos(true_phi(i)));
%     
% end

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
plot(historyX_required(1, end-P:end),  historyX_required(2, end-P:end), 'k-')
plot(historyX_ref(1, end-P:end),  historyX_ref(2, end-P:end), 'b--', 'LineWidth',2)
xlabel("X", 'fontsize',12)
ylabel("Y", 'fontsize',12)
title("Trajectory of Linear system", 'fontsize',14)
lgd = legend('Stochastic Trajectory', 'Predicted Trajectory', 'location', 'best')
lgd.FontSize = 12
hold off

print -depsc results.eps