clear; close all; 
ITER = 1e2; 
T = 0.02; 

%covariances: 
var_v1 =1e-2; 
var_v2 =1e-2; 
var_v3 = 1e-2; 
var_w1 = 1e-2;
var_w2 = 1e-2; 
Q_k = diag([var_v1 var_v2 var_v3]); 
W_k = diag([var_w1 var_w2]); 

%system 
F = eye(3);
b = 1/2; %width of chassis
H = [1 0 b; 1 0 -b]; 

%storage of values 
position = zeros(3, 1); 

%generate values 
x_k = [0; 0; 0];

%POSE variables: 
x_required = zeros(3, 1); 
x_ref = zeros(3, 1); 
truepose_history = zeros(3,1);
filteredpose_history = zeros(3,1); 

%create filter object
filter = ExtendedKF(@statetransition_f, @measurement_f,...
     @statetransition_j, @measurement_j,... 
    Q_k, W_k, T, x_k, false); 

for k = 1:ITER
    [Xpred, Ppred] = filter.predict(); 
    [Xcorr, Pcorr] = filter.correct();    
    
    %changing variable names:
    xhat_optimal = Xcorr; 
    x_k = filter.xk; 
    
    %calculate robot pose: 
    %xhat_optimal = [vx, vy, wn]
    x_required(3) = x_required(3) + T*xhat_optimal(3); %calculate phi
    Rf = [cos(x_required(3)) -sin(x_required(3)) 0; sin(x_required(3)) cos(x_required(3)) 0; 0 0 1]; %define rotation matrix:
    uc = Rf*xhat_optimal; %rotate the velocities 
    x_required(1:2) = [x_required(1) + uc(1)/uc(3)*(sin(x_required(3)+T*xhat_optimal(3)) - sin(x_required(3)));
                       x_required(2) - uc(2)/uc(3)*(cos(x_required(3)+T*xhat_optimal(3)) - cos(x_required(3)));
                        ];
    
    filteredpose_history(:, k) = x_required;  
    
    %required states: REFERENCE values:
    x_ref(3) = x_ref(3) + T*x_k(3); %calculate phi
    Rf = [cos(x_ref(3)) -sin(x_ref(3)) 0; sin(x_ref(3)) cos(x_ref(3)) 0; 0 0 1]; %define rotation matrix:
    uc = Rf*x_k; %rotate the velocities 
    x_ref(1:2) = [x_ref(1) + uc(1)/uc(3)*(sin(x_ref(3)+T*x_k(3)) - sin(x_ref(3)));
                       x_ref(2) - uc(2)/uc(3)*(cos(x_ref(3)+T*x_k(3)) - cos(x_ref(3)));
                        ]; 
                    
    truepose_history(:, k) = x_ref; 
                    
                    
end

 
figure(1)
subplot 311
hold on 
plot(filter.truehistory(1, :), 'red','linewidth', 2)
plot(filter.predhistory(1, :), 'b--', 'linewidth', 1)
title("x velocity ")
legend("True Output", "Filtered output")
hold off

% figure()
subplot 312
hold on 
plot(filter.truehistory(2, :), 'red','linewidth', 2)
plot(filter.predhistory(2, :), 'b--', 'linewidth', 1)
title("y velocity ")
legend("True Output", "Filtered output")
hold off

% figure()
subplot 313
hold on 
plot(filter.truehistory(3, :), 'red','linewidth', 2)
plot(filter.predhistory(3, :), 'b--', 'linewidth', 1)
title("Angular velocity ")
legend("True Output", "Filtered output")
hold off

figure(2)

hold on 
plot(filteredpose_history(1, :), filteredpose_history(2, :), 'red','linewidth', 1)
plot(truepose_history(1, :), truepose_history(2, :), 'b--', 'linewidth', 1)
title("Pose of robot")
legend("Filtered output", "True Output")
hold off
