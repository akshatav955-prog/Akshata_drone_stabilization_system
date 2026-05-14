clc;
clear;
close all;

%% DRONE STABILIZATION SYSTEM USING PID CONTROLLER

% Transfer Function of Drone
num = [1];
den = [1 2 5];
G = tf(num, den);

disp('Transfer Function of System:')
G

%% OPEN LOOP RESPONSE
figure;
step(G);
grid on;
title('Open Loop Response of Drone System');
xlabel('Time (seconds)');
ylabel('Amplitude');

% Performance Information
info_open = stepinfo(G);

%% PID CONTROLLER DESIGN
Kp = 10;
Ki = 5;
Kd = 2;

C = pid(Kp, Ki, Kd);

disp('PID Controller:')
C

%% CLOSED LOOP SYSTEM
T = feedback(C*G,1);

figure;
step(T);
grid on;
title('Closed Loop Response with PID Controller');
xlabel('Time (seconds)');
ylabel('Amplitude');

% Closed Loop Performance
info_closed = stepinfo(T);

%% COMPARISON OF OPEN LOOP AND CLOSED LOOP
figure;
step(G,'r',T,'b');
grid on;

legend('Open Loop','Closed Loop with PID');
title('Comparison of Open Loop and Closed Loop Responses');
xlabel('Time (seconds)');
ylabel('Amplitude');

%% DISTURBANCE REJECTION ANALYSIS
t = 0:0.01:10;

u = ones(size(t));          % Step input

disturbance = zeros(size(t));
disturbance(t>=5) = -0.5;   % Disturbance applied at t = 5 sec

input_signal = u + disturbance;

[y,t] = lsim(T,input_signal,t);

figure;
plot(t,y,'LineWidth',2);
grid on;

hold on;
xline(5,'r--','Disturbance Applied');

title('Drone Response with External Disturbance');
xlabel('Time (seconds)');
ylabel('Output Response');

legend('System Output');

%% CONTROL SIGNAL ANALYSIS
control_signal = lsim(C,input_signal,t);

figure;
plot(t,control_signal,'m','LineWidth',2);
grid on;

title('PID Controller Output (Control Effort)');
xlabel('Time (seconds)');
ylabel('Control Signal');

%% DISPLAY PERFORMANCE PARAMETERS
disp('-----------------------------------')
disp('OPEN LOOP PERFORMANCE')
disp(info_open)

disp('-----------------------------------')
disp('CLOSED LOOP PERFORMANCE')
disp(info_closed)

%% POLE-ZERO MAP
figure;
pzmap(T);
grid on;
title('Pole-Zero Map of Closed Loop System');

%% ROOT LOCUS
figure;
rlocus(G);
grid on;
title('Root Locus of Drone System');

%% BODE PLOT
figure;
bode(T);
grid on;
title('Bode Plot of Closed Loop System');