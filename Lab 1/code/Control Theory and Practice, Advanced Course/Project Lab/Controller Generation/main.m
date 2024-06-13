clear 
clc

% Laplace variable
s = tf('s');
%% Exercise 3.1 Static Decoupling: 
% G_tilde(s) = W_2(s)*G(s)*W_1(s)
% Choose W_2(s) = eye(n) 
% Choose W_1(s) = (G(0))^(-1)  Decoupling at steady state

% Minimum phase case
sys_G_mp = minphase;                                           % Minimum phase system

[num_G, den_G] = tfdata(sys_G_mp);                             % Extract system transfer functions 
G_mp = tf(num_G, den_G);

% Minimum phase case
sys_G_nmp = nonminphase;                                       % Minimum phase system

[num_G, den_G] = tfdata(sys_G_nmp);                       % Extract system transfer functions 
G_nmp = tf(num_G, den_G);

%% Performance Limitation analysis
fprintf('Minimum phase system: \n')
fprintf("The multivariable system has poles in: \n" )
disp(pole(sys_G_mp))
fprintf("The multivariable system has zeros in: \n" )
disp(tzero(sys_G_mp))
fprintf('Nonminimum phase system: \n')
fprintf("The multivariable system has poles in: \n" )
disp(pole(sys_G_nmp))
fprintf("The multivariable system has zeros in: \n" )
disp(tzero(sys_G_nmp))
%% Exercise 3.1.1: Static decoupling 
sys_G_tilde_mp = minreal(sys_G_mp*inv(dcgain(sys_G_mp)));       
sys_G_tilde_nmp = minreal(sys_G_nmp*inv(dcgain(sys_G_nmp)));       


% PLot the bode plot
figure;
bode(sys_G_tilde_mp, sys_G_tilde_nmp)                          % Off-diagonal terms should be almost 
                                                               % zero at steady-state                          
%% Exercise 3.1.2: Design a diagonal controller
omega_c_mp = 0.1;                                              % Desired cross-over frequency (mp) [rad/sec]
omega_c_nmp = 0.015;                                              % Desired cross-over frequency (nmp) [rad/sec]
pm = pi/3;                                                     % Desired phase margin

[num_G, den_G] = tfdata(sys_G_tilde_mp);                       % Extract system transfer functions 
G_tilde_mp = tf(num_G, den_G);                                 % Obtain transfer function matrix

[num_G_nmp, den_G_nmp] = tfdata(sys_G_tilde_nmp);                       % Extract system transfer functions 
G_tilde_nmp = tf(num_G_nmp, den_G_nmp);                                 % Obtain transfer function matrix

% Controller f1(s)
% Compute the arg(g11(i*wc))
[~, arg_g11_wc, ~] = bode(G_tilde_mp(1,1), omega_c_mp);
% Compute the parameters for controller l_11(s)
Ti1 = tan(pm - pi/2 - arg_g11_wc*pi/180)/omega_c_mp;           % Compute the time constant
% Construct open-loop for subsystem 1,1
l_11 = G_tilde_mp(1,1)*(1+ 1/(s*Ti1));
% Evaluate the gain at desired crossover frequency
[l_11_wc, ~, ~ ] = bode(l_11, omega_c_mp);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_1 = 1/l_11_wc;

% Controller f2(s)
% Compute the arg(g22(i*wc))
[~, arg_g22_wc, ~] = bode(G_tilde_mp(2,2), omega_c_mp);
% Compute the parameters for controller l_11(s)
Ti2 = tan(pm - pi/2 - arg_g22_wc*pi/180)/omega_c_mp;           % Compute the time constant
% Construct open-loop for subsystem 1,1
l_22 = G_tilde_mp(2,2)*(1+ 1/(s*Ti2));
% Evaluate the gain at desired crossover frequency
[l_22_wc, ~, ~ ] = bode(l_22, omega_c_mp);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_2 = 1/l_22_wc;

% Validate the design
figure;
bode(l_11*K_1, l_22*K_2)
legend('l_{11}', 'l_{22}')
% Construct the decentralized controller
f1_tilde = K_1*(1+1/(Ti1*s));
f2_tilde = K_2*(1+1/(Ti2*s));
F_tilde_mp = append(f1_tilde, f2_tilde);


% Construct the actual controller F(s) = (G(0))^(-1)*F_tilde(s)
F_mp = minreal(inv(dcgain(sys_G_mp))*F_tilde_mp);

% Controller f1(s)
% Compute the arg(g11(i*wc))
[~, arg_g11_wc, ~] = bode(G_tilde_nmp(1,1), omega_c_nmp);
% Compute the parameters for controller l_11(s)
Ti1 = tan(pm - pi/2 - arg_g11_wc*pi/180)/omega_c_nmp;           % Compute the time constant
% Construct open-loop for subsystem 1,1
l_11 = G_tilde_nmp(1,1)*(1+ 1/(s*Ti1));
% Evaluate the gain at desired crossover frequency
[l_11_wc, ~, ~ ] = bode(l_11, omega_c_nmp);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_1 = 1/l_11_wc;

% Controller f2(s)
% Compute the arg(g22(i*wc))
[~, arg_g22_wc, ~] = bode(G_tilde_nmp(2,2), omega_c_nmp);
% Compute the parameters for controller l_11(s)
Ti2 = tan(pm - pi/2 - arg_g22_wc*pi/180)/omega_c_nmp;           % Compute the time constant
% Construct open-loop for subsystem 1,1
l_22 = G_tilde_nmp(2,2)*(1+ 1/(s*Ti2));
% Evaluate the gain at desired crossover frequency
[l_22_wc, ~, ~ ] = bode(l_22, omega_c_nmp);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_2 = 1/l_22_wc;

% Validate the design
figure;
bode(l_11*K_1, l_22*K_2)
legend('l_{11}', 'l_{22}')
% Construct the decentralized controller
f1_tilde_nmp = K_1*(1+1/(Ti1*s));
f2_tilde_nmp = K_2*(1+1/(Ti2*s));
F_tilde_nmp = append(f1_tilde_nmp, f2_tilde_nmp);

% Construct the actual controller F(s) = (G(0))^(-1)*F_tilde(s)
F_nmp = minreal(inv(dcgain(sys_G_nmp))*F_tilde_nmp);
%% Exercise 3.1.3: Analyse the system for robustness

S = minreal(feedback(eye(2), sys_G_mp*F_mp));                % Sensitivity function
T = minreal(feedback(sys_G_mp*F_mp, eye(2)));                % Complementary Sensitivity function

S_nmp = minreal(feedback(eye(2), sys_G_nmp*F_nmp));                % Sensitivity function
T_nmp = minreal(feedback(sys_G_nmp*F_nmp, eye(2)));                % Complementary Sensitivity function

% Plot the singular values of the sensitvity functions
figure;
subplot(2,1,1)
sigma(S, T);
title('Singular values: Minimum phase')
legend('S', 'T')
grid on;
subplot(2,1,2)
sigma(S_nmp, T_nmp);
title('Singular values: Non-minimum phase')
legend('S_{nmp}', 'T_{nmp}')
grid on;

fprintf("The Hinf-norm of S in the minimum phase case is ")
display(hinfnorm(S))
fprintf("The Hinf-norm of T in the minimum phase case is ")
display(hinfnorm(T))

fprintf("The Hinf-norm of S in the non-minimum phase case is ")
display(hinfnorm(S_nmp))
fprintf("The Hinf-norm of T in the non-minimum phase case is ")
display(hinfnorm(T_nmp))
%% Exercise 3.1.4: Simulate the closed loop
G = sys_G_nmp;
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

%% Compare min and nonmin cases 
G = sys_G_mp;
F = F_mp;
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
title('Minimum phase case')

G = sys_G_nmp;
F = F_nmp;
% Simulate the system
open("closedloop.mdl")
sim("closedloop.mdl")

subplot(2,1,2)
hold all
plot(yout.Time, yout.Data(:,1), "LineWidth", 1.5)
plot(yout.Time, yout.Data(:,2), "LineWidth", 1.5)
title('Non-minimum phase case')
grid on
xlabel("Time [sec]")
ylabel("y")
legend("y_1", "y_2")

%% Dynamic Decoupling 
% G_tilde(s) = W_2(s)G(s)W_1(s)

% Minimum phase system
% Evaluate RGA at steady-state
fprintf('Minimum phase system: \n')
G0 = evalfr(G_mp, 0);
fprintf("The steady-state RGA is \n")
RGA_G0 = G0.*(inv(G0))';
disp(RGA_G0)

% Choose W_2 = eye(2) and W_1(s) = [w11 w12;w21 w22]
% W_1 is chosen s.t. G_tilde(s) = G(s)W_1(s) is diagonal, i.e.
% g_11(s)w_11(s) + g_12(s)w_12(s) = 0; HINT: w_11(s) = w_22(s) = 0
% g_21(s)w_21(s) + g_22(s)w_22(s) = 0
% --> w_12(s) = -g_12(s)/g_11(s)
% --> w_21(s) = -g_21(s)/g_22(s)
w_12 = -minreal(G_mp(1,2)/G_mp(1,1));
w_21 = -minreal(G_mp(2,1)/G_mp(2,2));

% Construct the decoupler W1
W1_mp = [tf(1,1), minreal(w_12); minreal(w_21), tf(1,1)];

% Construct decoupled system G_tilde = G*W1
G_tilde_mp = minreal(G_mp*W1_mp);

% Validate the decoupling
figure;
bode(G_tilde_mp)


% Non-minimum phase system
fprintf('Nonminimum phase system: \n')
G_nmp0 = evalfr(G_nmp, 0);
fprintf("The steady-state RGA is \n")
RGA_G_nmp0 = G_nmp0.*(inv(G_nmp0))';
disp(RGA_G_nmp0)

% Choose W_2 = eye(2) and W_1(s) = [w11 w12;w21 w22]
% W_1 is chosen s.t. G_tilde(s) = G(s)W_1(s) is diagonal, i.e.
% g_11(s)w_11(s) + g_12(s)w_12(s) = 0; HINT: w_11(s) = w_22(s) = 0
% g_21(s)w_21(s) + g_22(s)w_22(s) = 0
% --> w_12(s) = -g_12(s)/g_11(s)
% --> w_21(s) = -g_21(s)/g_22(s)
w_11 = minreal(-G_nmp(2,2)/G_nmp(2,1));
w_22 = minreal(-G_nmp(1,1)/G_nmp(1,2));

% Construct the decoupler W1
W1_nmp = [w_11, tf(1,1); tf(1,1), w_22];
% To make the weights proper we realize dynamic decoupling until approx. 10
% times the cross-over frequency omega_c_nmp
omega_c_nmp = 0.015;                                            % Desired cross-over frequency (nmp) [rad/sec]
W1_nmp = W1_nmp*10*omega_c_nmp/(s+ 10*omega_c_nmp);

% Construct decoupled system G_tilde = G*W1
G_tilde_nmp = minreal(G_nmp*W1_nmp);

% Validate the decoupling
figure;
bode(G_tilde_nmp)

%% Exercise 3.2.2: Design a diagonal controller
omega_c_mp = 0.1;                                              % Desired cross-over frequency (mp) [rad/sec]
pm = pi/3;                                                     % Desired phase margin

% Minimum phase system
% Controller f1(s)
% Compute the arg(g11(i*wc))
[~, arg_g11_wc, ~] = bode(G_tilde_mp(1,1), omega_c_mp);
% Compute the parameters for controller l_11(s)
Ti1 = tan(pm - pi/2 - arg_g11_wc*pi/180)/omega_c_mp;           % Compute the time constant
% Construct open-loop for subsystem 1,1
l_11 = G_tilde_mp(1,1)*(1+ 1/(s*Ti1));
% Evaluate the gain at desired crossover frequency
[l_11_wc, ~, ~ ] = bode(l_11, omega_c_mp);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_1 = 1/l_11_wc;

% Controller f2(s)
% Compute the arg(g22(i*wc))
[~, arg_g22_wc, ~] = bode(G_tilde_mp(2,2), omega_c_mp);
% Compute the parameters for controller l_11(s)
Ti2 = tan(pm - pi/2 - arg_g22_wc*pi/180)/omega_c_mp;           % Compute the time constant
% Construct open-loop for subsystem 1,1
l_22 = G_tilde_mp(2,2)*(1+ 1/(s*Ti2));
% Evaluate the gain at desired crossover frequency
[l_22_wc, ~, ~ ] = bode(l_22, omega_c_mp);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_2 = 1/l_22_wc;

% Validate the design
figure;
bode(l_11*K_1, l_22*K_2)
legend('l_{11}', 'l_{22}')
% Construct the decentralized controller
f1_tilde = K_1*(1+1/(Ti1*s));
f2_tilde = K_2*(1+1/(Ti2*s));
F_tilde_mp = append(f1_tilde, f2_tilde);


% Construct the actual controller F(s) = W1(s)*F_tilde(s)
F_mp = minreal(W1_mp*F_tilde_mp);

% Non-minimum phase system
% Controller f1(s)
% Compute the arg(g11(i*wc))
[~, arg_g11_wc, ~] = bode(G_tilde_nmp(1,1), omega_c_nmp);
% Compute the parameters for controller l_11(s)
Ti1 = tan(pm - pi/2 - arg_g11_wc*pi/180)/omega_c_nmp;           % Compute the time constant
% Construct open-loop for subsystem 1,1
l_11 = G_tilde_nmp(1,1)*(1+ 1/(s*Ti1));
% Evaluate the gain at desired crossover frequency
[l_11_wc, ~, ~ ] = bode(l_11, omega_c_nmp);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_1 = 1/l_11_wc;

% Controller f2(s)
% Compute the arg(g22(i*wc))
[~, arg_g22_wc, ~] = bode(G_tilde_nmp(2,2), omega_c_nmp);
% Compute the parameters for controller l_11(s)
Ti2 = tan(pm - pi/2 - arg_g22_wc*pi/180)/omega_c_nmp;           % Compute the time constant
% Construct open-loop for subsystem 1,1
l_22 = G_tilde_nmp(2,2)*(1+ 1/(s*Ti2));
% Evaluate the gain at desired crossover frequency
[l_22_wc, ~, ~ ] = bode(l_22, omega_c_nmp);
% Choose K1 s.t. the desired cross-over frequency is obtained
K_2 = 1/l_22_wc;

% Validate the design
figure;
bode(l_11*K_1, l_22*K_2)
legend('l_{11}', 'l_{22}')
% Construct the decentralized controller
f1_tilde_nmp = K_1*(1+1/(Ti1*s));
f2_tilde_nmp = K_2*(1+1/(Ti2*s));
F_tilde_nmp = append(f1_tilde_nmp, f2_tilde_nmp);

% Construct the actual controller F(s) = (G(0))^(-1)*F_tilde(s)
F_nmp = minreal(W1_nmp*F_tilde_nmp);

%% Exercise 3.2.3: Analyse the system for robustness

S = minreal(feedback(eye(2), G_mp*F_mp));                % Sensitivity function
T = minreal(feedback(G_mp*F_mp, eye(2)));                % Complementary Sensitivity function

S_nmp = minreal(feedback(eye(2), minreal(G_nmp*F_nmp))); % Sensitivity function
T_nmp = minreal(feedback(G_nmp*F_nmp, eye(2)));          % Complementary Sensitivity function

% Plot the singular values of the sensitvity functions
figure;
subplot(2,1,1)
sigma(S, T);
title('Singular values: Minimum phase')
legend('S', 'T')
grid on;
subplot(2,1,2)
sigma(S_nmp, T_nmp);
title('Singular values: Non-minimum phase')
legend('S_{nmp}', 'T_{nmp}')
grid on;

fprintf("The Hinf-norm of S in the minimum phase case is ")
display(hinfnorm(S))
fprintf("The Hinf-norm of T in the minimum phase case is ")
display(hinfnorm(T))

fprintf("The Hinf-norm of S in the non-minimum phase case is ")
display(hinfnorm(S_nmp))
fprintf("The Hinf-norm of T in the non-minimum phase case is ")
display(hinfnorm(T_nmp))


%% Exercise 3.2.4: Simulate the closed loop
G = sys_G_nmp;
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
sgtitle('Minimum phase case')

%% 3.3 Glover-McFarlane robust loop-shaping
% Minimum phase case: 
% Here we choose the results from dynamic decoupling

% Nominal loop gain
L0_mp = minreal(G_mp*F_mp);
L0_nmp = minreal(G_nmp*F_nmp);
alpha_mp = 1.1;                            % According to hint
alpha_nmp = 1.2;                           % According to hint
%% 3.3.1 Verify the decoupling process
figure;
bode(L0_mp);
title('Minimum phase case')

figure;
bode(L0_nmp);
title('Non-minimum phase case')

%% 3.3.2 Compute robustified controller
[Fr_mp, alpha_mp] = rloop(L0_mp, alpha_mp);
[Fr_nmp, alpha_nmp] = rloop(L0_nmp, alpha_mp);

% Construct overall robustified controller
F_robust_mp = F_mp*Fr_mp;
F_robust_nmp = F_nmp*Fr_nmp;
%% 3.3.3 Plot the singular values of the sensitvity functions and compare

S_robust = minreal(feedback(eye(2), G_mp*F_robust_mp));                % Sensitivity function
T_robust = minreal(feedback(G_mp*F_robust_mp, eye(2)));                % Complementary Sensitivity function

S_robust_nmp = minreal(feedback(eye(2), minreal(G_nmp*F_robust_nmp))); % Sensitivity function
T_robust_nmp = minreal(feedback(G_nmp*F_robust_nmp, eye(2)));          % Complementary Sensitivity function


figure;
subplot(2,1,1)
sigma(S, T, S_robust, T_robust);
title('Singular values: Minimum phase')
legend('S', 'T', 'S_{GM}', 'T_{GM}')
ylim([-30 5])
xlim([0.0001 100])
grid on;
subplot(2,1,2)
sigma(S_nmp, T_nmp, S_robust_nmp, T_robust_nmp);
title('Singular values: Non-minimum phase')
legend('S', 'T', 'S_{GM}', 'T_{GM}')
ylim([-30 5])
xlim([0.0001 100])
grid on;

fprintf("The Hinf-norm of the robustified S in the minimum phase case is ")
display(hinfnorm(S_robust))
fprintf("The Hinf-norm of the robustified T in the minimum phase case is ")
display(hinfnorm(T_robust))

fprintf("The Hinf-norm of the robustified S in the non-minimum phase case is ")
display(hinfnorm(S_robust_nmp))
fprintf("The Hinf-norm of the robustified T in the non-minimum phase case is ")
display(hinfnorm(T_robust_nmp))

%% Exercise 3.3.4: Simulate the closed loop
G = sys_G_nmp;
F = F_robust_nmp;
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
sgtitle('Non-minimum phase case')

%% Save controllers

F = ss(F_mp,'min');
[A,B,C,D] = ssdata(F);
save 'Nominal-Decentralized-Controller_min.MAT' A B C D

F = ss(F_nmp,'min');
[A,B,C,D] = ssdata(F);
save 'Nominal_Decentralized_Controller_nonmin.MAT' A B C D

F = ss(F_robust_mp,'min');
[A,B,C,D] = ssdata(F);
save 'Glover-McFarlane_min.MAT' A  B  C  D

F = ss(F_robust_nmp,'min');
[A,B,C,D] = ssdata(F);
save 'Glover-McFarlane_nonmin.MAT' A  B C D



