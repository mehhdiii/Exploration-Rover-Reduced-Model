function [v,omega, phi] = inv_kinematic_from_trajectory(T,N)
% Differential drive WMR Trajectory creator

T = 0.001; %Sampling time
W = 1/2; %m. Width of Robot
r = 1/4; %m. Radius of tires

% Create Trajectory for WMR:
N=400;t=linspace(-pi,pi,N);
f1=1;x=8*sin(f1*t);f2=2;y=8*sin(f2*t);
phi =zeros(1, N);

%initialize Inverse kinematics velocities
omega = zeros(1, N); v = zeros(1, N); 
vL = zeros(1, N); vR = zeros(1, N); 
omegaL = zeros(1, N); omegaR = zeros(1, N); 

%initialize resulting forward kinematic variables: 
x_f = zeros(1, N); y_f = zeros(1, N); phi_f = zeros(1, N); 

%plotting initializations: 
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'figures/testAnimated.gif';
h = figure; 

for n = 2:N-1
    
    phi(n) = atan2(y(n)-y(n-1), x(n)-x(n-1)); 

    %calculating inverse kinematics variables: 
    mu = 1/2*(sin(phi(n))*(y(n+1)-y(n))+cos(phi(n)*(x(n+1)-x(n))))...
        /(cos(phi(n))*(y(n+1)-y(n))-sin(phi(n))*(x(n+1)-x(n))); 
    x_m = (x(n)+x(n+1))/2; 
    y_m = (y(n)+y(n+1))/2; 
    
    x_star = x_m - mu/2 * (y(n+1) - y(n)); 
    y_star = y_m + mu/2 * (x(n+1)-x(n)); 
    
    R_n = sqrt((x(n) - x_star)^2 + (y(n)-y_star)^2); 
    theta_1 = atan2((y(n)-y_star), (x(n)-x_star)); 
    theta_2 = atan2((y(n+1)-y_star), (x(n+1)-x_star)); 
    del_phi = wrapToPi(theta_1 - theta_2); 
    
    %resulting Inv-Kinematics velocities: 
    omega(n) = del_phi/T; 
    v(n) = R_n*abs(omega(n));  
    vL(n) = (R_n-1/2 *W)*omega(n); 
    vR(n) = (R_n+1/2 *W)*omega(n); 
    omegaL(n) = vL(n)/r; 
    omegaR(n) = vR(n)/r; 
    
    %forward Kinematics: circular velocity motion model/Exact integration
    %model 
    x_f(n+1) = x_f(n) + (v(n)/omega(n))*(-sin(phi(n))+sin(phi(n)+omega(n)*T));
    y_f(n+1) = y_f(n) + (v(n)/omega(n))*(cos(phi(n))-cos(phi(n)+omega(n)*T));
    
    %Alternately: Model obtained from trapezoidal integration: 
    % x_f(n+1) = x_f(n) + (v(n)*T/2)*(cos(phi_n)+cos(phi_n+omega(n)*T));
    % y_f(n+1) = y_f(n) + (v(n)*T/2)*(sin(phi_n)+sin(phi_n+omega(n)*T));

%     plot(x_f(1:n), y_f(1:n), 'g-', 'linewidth', 6); 
%     axis([-9  9 -9  9])
%     hold on; plot(x, y, 'k-', 'linewidth', 1); 
%     legend('Calculated path', 'Desired path')
%     drawnow;  
    
%     %create GIF
%     frame = getframe(h); 
%     im = frame2im(frame); 
%     [imind,cm] = rgb2ind(im,256); 
%     % Write to the GIF File 
%     if n == 2
%       imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%     else 
%       imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%     end 
end

figure()

subplot 211
plot(t,omega,'linewidth',2);
title('Angular Velocity')
xlabel("t")
ylabel('\omega')
subplot 212
plot(t, v,'linewidth',2)

title('Linear velocity')
xlabel("t")
ylabel("v")


figure()
hold on 
 
plot(x_f, y_f, 'linewidth', 6); 
plot(x,y,'linewidth',2);
legend('Calculated Path', 'Original Path')
hold off

% print -deps figures/OutputFig

end

