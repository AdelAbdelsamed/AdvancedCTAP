%% Control Theory and Practice Advanced Course
% Computer Lab 2
clear
clc
%%  3.1 Poles, zeros and RGA
% Define the minimum phase system
sysmp = minphase;
% Define the non-minimum phase system
sysnonmp = nonminphase;

[num_G, den_G] = tfdata(sysmp); % Extract system transfer functions 
G = tf(num_G, den_G);           % Obtain transfer function matrix

[num_G_nmp, den_G_nmp] = tfdata(sysnonmp); % Extract system transfer functions 
G_nmp = tf(num_G_nmp, den_G_nmp);           % Obtain transfer function matrix
% Show numerator and denominator coefficients
fprintf('Minimum phase system: \n')
fprintf("The numerator coefficients of G are: \n")
disp(num_G)
fprintf("The denominator coefficients of G are: \n")
disp(den_G)
fprintf('Nonminimum phase system: \n')
fprintf("The numerator coefficients of G are: \n")
disp(num_G_nmp)
fprintf("The denominator coefficients of G are: \n")
disp(den_G_nmp)

% Poles
fprintf('Minimum phase system: \n')
fprintf("The poles of the element G(1,1) are: \n")
disp(pole(G(1,1)))
fprintf("The poles of the element G(1,2) are: \n")
disp(pole(G(1,2)))
fprintf("The poles of the element G(2,1) are: \n")
disp(pole(G(2,1)))
fprintf("The poles of the element G(2,2) are: \n")
disp(pole(G(2,2)))
fprintf('Nonminimum phase system: \n')
fprintf("The poles of the element G(1,1) are: \n")
disp(pole(G_nmp(1,1)))
fprintf("The poles of the element G(1,2) are: \n")
disp(pole(G_nmp(1,2)))
fprintf("The poles of the element G(2,1) are: \n")
disp(pole(G_nmp(2,1)))
fprintf("The poles of the element G(2,2) are: \n")
disp(pole(G_nmp(2,2)))

%% Ex. 3.1.2
fprintf('Minimum phase system: \n')
fprintf("The multivariable system has poles in: \n" )
disp(pole(sysmp))
fprintf("The multivariable system has zeros in: \n" )
disp(tzero(sysmp))
fprintf('Nonminimum phase system: \n')
fprintf("The multivariable system has poles in: \n" )
disp(pole(sysnonmp))
fprintf("The multivariable system has zeros in: \n" )
disp(tzero(sysnonmp))

%% Ex. 3.1.3
% Plot the singular values
figure;
sigma(G, G_nmp)
grid on;

fprintf("The H-infinity norm of the minimum phase G is \n")
disp(hinfnorm(G))
fprintf("The H-infinity norm of the nonminimum phase G is \n")
disp(hinfnorm(G_nmp))

%% Ex. 3.1.4
% Evaluate at steady-state
fprintf('Minimum phase system: \n')
G0 = evalfr(G, 0);
fprintf("The steady-state RGA is \n")
RGA_G0 = G0.*(inv(G0))';
disp(RGA_G0)
fprintf('Nonminimum phase system: \n')
G_nmp0 = evalfr(G_nmp, 0);
fprintf("The steady-state RGA is \n")
RGA_G_nmp0 = G_nmp0.*(inv(G_nmp0))';
disp(RGA_G_nmp0)

%% Ex. 3.1.5
% PLot the step response
figure;
step(G, G_nmp)

%% Ex. 3.2.1 Minimum Phase
% Desired phase margin 
pm = pi/3;
% Desired crossover frequency
wc = 0.1;
% Laplace variable
s = tf('s');

% Controller f1(s)
% Compute the arg(g11(i*wc))
[~, arg_g11_wc, ~] = bode(G(1,1), wc);
% Compute the parameters for controller l_11(s)
Ti1 = tan(pm - pi/2 - arg_g11_wc*pi/180)/wc;           % Compute the time constant
% Construct open-loop for subsystem 1,1
l_11 = G(1,1)*(1+ 1/(s*Ti1));
% Evaluate the gain at desired crossover frequency
[l_11_wc, ~, ~ ] = bode(l_11, wc);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_1 = 1/l_11_wc;

% Controller f2(s)
% Compute the arg(g22(i*wc))
[~, arg_g22_wc, ~] = bode(G(2,2), wc);
% Compute the parameters for controller l_11(s)
Ti2 = tan(pm - pi/2 - arg_g22_wc*pi/180)/wc;           % Compute the time constant
% Construct open-loop for subsystem 1,1
l_22 = G(2,2)*(1+ 1/(s*Ti2));
% Evaluate the gain at desired crossover frequency
[l_22_wc, ~, ~ ] = bode(l_22, wc);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_2 = 1/l_22_wc;

% Construct the decentralized controller
f1 = K_1*(1+1/(Ti1*s));
f2 = K_2*(1+1/(Ti2*s));
F = append(f1, f2);

% Open-loop 
L = G*F;

% Plot the bode fiagram
figure;
bode(minreal(L))

%% Ex. 3.2.1 Nonminimum Phase
% Desired phase margin 
pm = pi/3;
% Desired crossover frequency
wc_nmp = 0.02;
% Laplace variable
s = tf('s');

% Controller f1(s)
% Compute the arg(g12(i*wc))
[~, arg_g12_wc, ~] = bode(G_nmp(1,2), wc_nmp);
% Compute the parameters for controller l_12(s)
Ti1 = tan(pm - pi/2 - arg_g12_wc*pi/180)/wc_nmp;           % Compute the time constant
% Construct open-loop for subsystem 1,2
l_12 = G_nmp(1,2)*(1 + 1/(s*Ti1));
% Evaluate the gain at desired crossover frequency
[l_12_wc, ~, ~ ] = bode(l_12, wc_nmp);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_1 = 1/l_12_wc;

% Controller f2(s)
% Compute the arg(g21(i*wc))
[~, arg_g21_wc, ~] = bode(G_nmp(2,1), wc_nmp);
% Compute the parameters for controller l_21(s)
Ti2 = tan(pm - pi/2 - arg_g21_wc*pi/180)/wc_nmp;           % Compute the time constant
% Construct open-loop for subsystem 2,1
l_21 = G_nmp(2,1)*(1 + 1/(s*Ti2));
% Evaluate the gain at desired crossover frequency
[l_21_wc, ~, ~ ] = bode(l_21, wc_nmp);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_2 = 1/l_21_wc;

% Construct the decentralized controller
f1 = K_1*(1+1/(Ti1*s));
f2 = K_2*(1+1/(Ti2*s));
F_nmp = [tf(0), f1;f2, tf(0)];

% Open-loop 
L_nmp = G_nmp*F_nmp;

% Plot the bode fiagram
figure;
bode(L_nmp)

%% Exercise 3.2.2
% Sensitivity functions for minimum phase systems
S = minreal(feedback(eye(2), L));          % Sensitvity Function
T = minreal(feedback(L, eye(2)));          % Complementary Sensitivity Function

% Sensitivity functions for non-minimum phase systems
S_nmp = minreal(feedback(eye(2), L_nmp));          % Sensitvity Function
T_nmp = minreal(feedback(L_nmp, eye(2)));          % Complementary Sensitivity Function

% Plot the singular values of the sensitvity functions
figure;
sigma(S, T);
grid on;

fprintf("The Hinf-norm of S is ")
display(hinfnorm(S))
fprintf("The Hinf-norm of T is ")
display(hinfnorm(T))

figure;
sigma(S_nmp, T_nmp);
grid on;

fprintf("The Hinf-norm of S is ")
display(hinfnorm(S_nmp))
fprintf("The Hinf-norm of T is ")
display(hinfnorm(T_nmp))
%% Exercise 3.2.3
G = G_nmp;
F = F_nmp;
% Simulate the system
open("closedloop.mdl")
sim("closedloop.mdl")

% Plot the step responses
figure;
subplot(2,1,1)
hold all
plot(yout.Time, yout.Data(:,1), "LineWidth", 1.5)
plot(yout.Time, yout.Data(:,2), "LineWidth", 1.5)
grid on
xlabel("Time [sec]")
ylabel("y")
legend("y_1", "y_2")
subplot(2,1,2)
hold all
plot(uout.Time, uout.Data(:,1), "LineWidth", 1.5)
plot(uout.Time, uout.Data(:,2), "LineWidth", 1.5)
grid on
xlabel("Time [sec]")
ylabel("u")
legend("u_1", "u_2")


