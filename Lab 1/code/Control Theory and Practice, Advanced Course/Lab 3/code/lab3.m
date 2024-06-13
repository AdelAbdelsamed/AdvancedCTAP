%% EL2520: Control Theory and Practice, Advanced Course
% Computer Lab 3
open("servo1.mdl")

% Define the Laplace Variable
s = tf("s");

% Define the plant transfer function
G = 1e4*(s + 2)/((s+3)*(s+100)^2);

%% Shaping the sensivity function S
% Initial design for weights (from lecture script): Ws = (1/M_s) + w_bs/s
M_s = 1.5;                 % Maximum sensitivty peak
w_bs = 100*pi;              % Bandwidth of sensitivity function
Ws = (1/M_s) + w_bs/s;    % Weight transfer function

% Alternative design for weights (from Multivariable Feedback Control P.58)
% Ws = (s/M_s + w_bs)/(s + w_bsA)
A = 0.01; % Steady-state error 
Ws2 = ((s/M_s) + w_bs)/(s + w_bs*A);

% Alternative design for weights (from Multivariable Feedback Control P.58)
% Immproved Performance: Slope 2
% Ws = (s/sqrt(M_s) + w_bs)^2/(s + w_bs*sqrt(A))^2
w_bs3 = 600;
Ws3 = (s/sqrt(M_s) + w_bs3)^2/(s + w_bs3*sqrt(A))^2;

figure;
bode(Ws, Ws2, Ws3)
legend("Script", "MFC", "MFC2")

%% Hint: Place poles at s= -epsilon +-i*sqrt(w^2 - epsilon^2)
epsilon = 0.01;
w = 100*pi;
Ws4 = 1/(s^2+2*epsilon*s + w^2);

figure;
bode(Ws4)
legend("Hint")

%% Exercise 4.1.(ii) Design the weights using the GUI 
Hinf(G);
%% Simulate the designed controller
Fsim = F;   
Gsim = G;
macro;          % Run the simulation
%% To get the same attentuation using a P-controller
% |S(iw_des)| \apporx |P*G(iw_des)|
wd = 100*pi;                % Frequency of disturbance
G_at_wd = bode(G,wd);   % Determine frequency at w = 50 rad/sec
P = (8*10^(-4))/G_at_wd;      % Proportional gain achieving the same disturbance rejection
P_db = 20*log(P);        % Amplification required in dB
%% Exercise 4.2
G_per =minreal(G*((s-1)/(s+2)));  % Perturbed System 
% 4.2.(i) Performance Limitation: w_bs <= 0.5 (M_s = 2); hence controller from
% 4.1.(ii) won't yield acceptable control

% 4.2.(ii): Validate the results in simulation
Fsim = F;
Gsim = G_per;
macro;

%% 4.2.(iii) Check for robust stability
T = minreal(feedback(G*F, 1));    % Compute the complementary sensitivity function
% Condition for robust stability: T and Delta_G(s) are stable &  
% hinfnorm(T*Delta_G) <= 1 with Delta_G = -3/(s+2)
DeltaG = -3/(s+2);

% Plot the condition
win = {10^-5,10^5};
figure;
[mag,phase,wout] = bode(T, win);
[mag2,phase2,wout2] = bode(1/DeltaG, win);
loglog(wout,squeeze(mag), 'LineWidth', 1.5)
hold on
loglog(wout2,squeeze(mag2), 'LineWidth', 1.5)
grid on
h = legend("T","$(\Delta_{G}) ^{-1}$");
set(h,'Interpreter','latex')

fprintf("hinfnorm of T*Delta_G is %d! \n", hinfnorm(T*DeltaG))   % Robust stability not guaranteed!

%% Design the W_T
% Initial Design of W_T = ((1/M_T) + s/w_bT) (from Lecture script)
% |W_T| <= |DeltaG| for robust stability!

% Paramters of weight design
w_bT = 4;
M_T = 4;
% Weight design
WT = (s/w_bT + 1/M_T);
% Plot WT and Delta_G
figure;
bode(WT, 1/DeltaG)
legend("WT", "(\Delta_G)^{-1}")
%% Design and validation
% 4.2.(iv): Validate the results in simulation using the perturbed plant
Fsim = F;
Gsim = G_per;   % Use perturbed plant in simulation
macro;
%%
figure;
bode(F*feedback(1, F*G))
