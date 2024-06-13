%% Basics 
clear
clc

% Laplace Variable
s = tf('s');

% System 
G = 3*(-s+1)/((5*s+1)*(10*s+1));

% Closed-loop system
Gcl = feedback(G, 1);
pole(Gcl)

% Plot the Bode plot
figure;
bode(G)

% Open-loop 
[Gm,Pm,Wcg,Wcp] = margin(G);

% Phase before Controller
[~, Phi_G_wc] = bode(G, 0.4);
%% 4.1.1 Leag-Lead Cmpensator

% Desired closed-loop paramters
wc = 0.4;
phi_m = 50;
%phi_m = 30;

% Lag controller parmaters 
phi_rec = phi_m -(Phi_G_wc -360 + 180) + 8;
%beta = 0.53;
beta = 0.22;
tau_D = 1/(wc*sqrt(beta));

% Lead controller parameters
tau_I = 10/wc;
gamma = 0;

% Lead compensator
F_lead = (tau_D*s + 1)/(beta*tau_D*s + 1);

% Lag compensator
F_lag = (tau_I*s + 1)/(tau_I*s + gamma);

% Get gain at desired cross-over frequency 
[Mag_G_wc, Phi_G_wc] = bode(F_lead*G, wc);

% Final Lead Compensator
F_lead = (1/Mag_G_wc)*F_lead;

% Open-loop 
G_ol = F_lead*F_lag*G;

% Validate the open-loop gain design
[Gm_ol,Pm_ol,Wcg_ol,Wcp_ol] = margin(G_ol);

% Plot the bode plot
figure;
bode(G_ol)

% PLot the step response
figure;
G_cl = minreal(feedback(G_ol, 1));
step(G_cl)

%% 4.1.2 System characteristics

% Resonance Peak M_T
T = G_cl;
M_T = hinfnorm(T);

% PLot the complementary sensitivity function
figure;
bode(T)

% Bandwidth of Complementary Sensitivity Function
w_BT = bandwidth(T);

% PLot the step response
figure;
step(T)
% Rise Time
stepinfo(T)

