clear 
%clc

% Laplace Variable 
s = tf('s');

% Transfer functions
G = 20/(((1/(20)^2)*s^2 + (1/20)*s + 1)*(s+1));  % r -> y
Gd = 10/(s+1);                                   % d -> y

%% Exercise 4.2.1
% Plot Gd to determine frequencies where |Gd| > 1
figure;
bode(Gd)
title("Bode diagram of Gd")
[~,~, ~, wcG] = margin(Gd);

% Desired cross-over frequency can be approximated to 10 rad/sec
wc = 10;

% Initial Controller using an inverse of the model s.t. L \approx w_c/s
Fy_init = (1/G)*wc/s;

% Add high frequency dynamics (above desired cross-over) to make controller proper
F_high_freq_poles = 1/(0.0001*s +1)^2; 
Fy_init_proper = Fy_init* F_high_freq_poles;

% Plot the "ideal" and approximated controller
figure;
bode(Fy_init,Fy_init_proper)
title("Approximation of ideal controller")
legend("Ideal Controller", "Approximated Controller")

% Transfer Function from d to y
T_d_to_y = minreal(feedback(1, Fy_init_proper*G))*Gd;
% Transfer Function from r to y
T_r_to_y = minreal(feedback(Fy_init_proper*G, 1));

% Plot the response to a unit distrubance and a unit reference
figure;
subplot(1,2,1)
step(T_d_to_y);
title("Unit step distrubance")
subplot(1,2,2)
step(T_r_to_y)
title("Unit step reference")

% Evaluate loop shapes by confirming that |Fy| > |G^(-1)Gd| (Here this is not satisfied, hence disturbance rejection performance is suboptimal)
figure;
subplot(2,1,1)
bode(Fy_init_proper,G^(-1)*Gd)
legend('Fy', 'Gd*G^(-1)')
subplot(2,1,2)
bode(Gd, Fy_init_proper*G)
legend('Gd', 'L')

%% Exercise 4.2.2
% Approximate G^(-1)*Gd with a constant 0.5 
figure;
bode(Gd/G, (Gd/G)/(s/200+1)^2);
title("Term to be approximated")

% Plot the open-loop with the P-controller
L_improper = Gd;                     % Improper
L1 = 0.5*G;                            % Approximation #1
figure;
bode(L_improper, L1)
title("Open-loop with P-Controller")

%% Integral action effective frequency
wI = 0.55*wcG;

% Modified controller
F_integrator = (s + wI)/s;
%Fy_improper = F_integrator*Gd/G;       % Improper
%Fy1 = F_integrator*0.94/2;             % Approximation #1
Fy2 = F_integrator*(Gd/G)/((s/(10*wI) +1)^2);               % Approximation #2

Fy = Fy2;     % Selcet Controller
% Open-loop gain
L = Fy*G;

% Plot the open-loop with the added PI-Controller
figure;
bode(L)
title("Open-loop with PI-Controller")

% Evaluate loop shapes by confirming that |Fy| > |G^(-1)Gd|
% figure;
% subplot(2,1,1)
% bode(Fy,G^(-1)*Gd)
% legend('Fy', 'Gd*G^(-1)')
% subplot(2,1,2)
% bode(Gd, L)
% legend('Gd', 'L')

% Transfer Function from d to y
T_d_to_y = minreal(feedback(1, Fy*G))*Gd;
% Transfer Function from d to y
T_r_to_y = minreal(feedback(Fy*G, 1));

% Plot the response to a unit distrubance and a unit reference
figure;
subplot(1,2,1)
step(T_d_to_y);
title("Unit step distrubance")
subplot(1,2,2)
step(T_r_to_y)
title("Unit step reference")

%% Exercise 4.2.3

% Desired closed-loop paramters
phi_m = 60;
% Obtain phase margin of pre-lead loop
[~, PM,~,wCLead] = margin(L);
% Lead controller parmaters 
phi_rec = phi_m - PM + 8;       % -> Look at table for beta!
%beta = 0.37;                     % From Table (for approx #1)
beta = 0.75;                     % From Table (for approx #2)
tau_D = 1/(wCLead*sqrt(beta));  

%% Lead compensator
F_lead = (tau_D*s + 1)/(beta*tau_D*s + 1);
% Get gain at desired cross-over frequency 
[Mag_G_wc, Phi_G_wc] = bode(F_lead*Fy*G, wCLead);
% Final Lead Compensator
F_lead = (1/Mag_G_wc)*F_lead;

% Open-loop 
Fy_lead = minreal(F_lead*Fy);
L = Fy_lead*G;

% Validate the open-loop gain design
[Gm_ol,Pm_ol,Wcg_ol,Wcp_ol] = margin(L);

% Design the pre-filter
tau = 0.12;
Fr = 1/(tau*s + 1);

% Closed-loop transfer function from r to y
T_r_to_y = Fr*minreal(feedback(L, 1));
% Transfer Function from d to y
T_d_to_y = minreal(feedback(1, L))*Gd;


% Obtain step response data 
stepinfo(T_r_to_y)
figure;
step(T_r_to_y)

%% Plot the response to a unit distrubance and a unit reference
figure;
subplot(1,2,1)
step(T_d_to_y);
title("Unit step distrubance")
subplot(1,2,2)
step(T_r_to_y)
title("Unit step reference")

% Consider the transfer function from inputs to u
S = minreal(feedback(1,L));
T = minreal(feedback(L,1));

% Closed-loop transfer function from r to u
T_r_to_u = Fr*Fy_lead*S;
% Transfer Function from d to y
T_d_to_u = -Fy_lead*S*Gd;

% Plot the control input to a unit distrubance and a unit reference
figure;
subplot(1,3,1)
step(T_d_to_u);
title("Unit step distrubance: Control input")
subplot(1,3,2)
step(T_r_to_u)
title("Unit step reference: Control input")
subplot(1,3,3)
step(T_r_to_u + T_d_to_u)
title("Unit step: Combined (reference + disturbance)")

%% Exercise 4.2.3
 % Plot the Bode diagrams of the sensivity function and complementary
 % sensitivity function
 figure;
 bode(S, T)
 title('Sensitivity function S and Complementary Sensitivity function T')
 legend('S', 'T')





