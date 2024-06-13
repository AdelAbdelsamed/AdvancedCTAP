%% Project lab

% Operating levels of tanks
h10 = 17;      % i cm (nedre vänstra tanken)
h20 = 20;      % i cm (nedre högra tanken)
h30 = 8.5;      % i cm (övre vänstra tanken)
h40 = 11;      % i cm (övre högra tanken)

% Operating voltage of pumps
u10 = 7.5;     % i V
u20 = 7.5;     % i V

% Operating actuator proportional constants
k1 = 2.59;     % i cm^3/(Vs)
k2 = 2.54;     % i cm^3/(Vs)

% Valve settings
gam1 = 0.375;
gam2 = 0.375;

% Tank Area
A = 15.52;     % i cm^2
% Acceleration of gravity
g = 981;        % cm/s^2

% Determine the outlet hole areas a_i
A_lin = [-sqrt(2*g*h10)/A, 0, sqrt(2*g*h30)/A, 0;
         0, -sqrt(2*g*h20)/A, 0, sqrt(2*g*h40)/A;
         0, 0, -sqrt(2*g*h30)/A, 0;
         0, 0, 0, -sqrt(2*g*h30)/A
         ];

b_lin = [-k1*gam1*u10/A;
         -k2*gam2*u20/A;
         -k2*(1-gam2)*u20/A;
         -k1*(1-gam1)*u10/A];

a_vec = A_lin\b_lin;

%% Obtain data from figure
fig = gcf;
dataObjs = findobj(fig,'-property','YData');
Input1 = dataObjs(1).YData;
Input2 = dataObjs(2).YData;
Tank1 = dataObjs(7).YData;
Tank2 = dataObjs(8).YData;
dataObjs = findobj(fig,'-property','xData');
time = dataObjs(1).XData;


%% PLot the results Input 1
indx = (time > 1450) & (time <1500);
figure;
plot(time(indx),Input1(indx), 'LineWidth', 1.5)
hold all
plot(time(indx),Input2(indx), 'LineWidth', 1.5)
plot(time(indx),Tank1(indx), 'LineWidth', 1.5)
plot(time(indx),Tank2(indx), 'LineWidth', 1.5)
grid on
title("Step response in input 1")
legend('Input 1', 'Input 2', 'Tank1', 'Tank 2')


%% PLot the results Input 2
indx = (time > 630) & (time <680);
figure;
plot(time(indx),Input1(indx), 'LineWidth', 1.5)
hold all
plot(time(indx),Input2(indx), 'LineWidth', 1.5)
plot(time(indx),Tank1(indx), 'LineWidth', 1.5)
plot(time(indx),Tank2(indx), 'LineWidth', 1.5)
grid on
title("Step response in input 1")
legend('Input 1', 'Input 2', 'Tank1', 'Tank 2')
xlim([630 680])
ylim([-5 51])
%% Combine the plots
indx = (time > 475) & (time <540);
indx2 = (time > 630) & (time <680);
figure
subplot(2,1,1)
plot(time(indx),Input1(indx), 'LineWidth', 1.5)
hold all
plot(time(indx),Input2(indx), 'LineWidth', 1.5)
plot(time(indx),Tank1(indx), 'LineWidth', 1.5)
plot(time(indx),Tank2(indx), 'LineWidth', 1.5)
grid on
title("Step response in input 1")
legend('Input 1', 'Input 2', 'Tank1', 'Tank 2')
xlim([475 540])
ylim([0 51])
subplot(2,1,2)
plot(time(indx2),Input1(indx2), 'LineWidth', 1.5)
hold all
plot(time(indx2),Input2(indx2), 'LineWidth', 1.5)
plot(time(indx2),Tank1(indx2), 'LineWidth', 1.5)
plot(time(indx2),Tank2(indx2), 'LineWidth', 1.5)
grid on
title("Step response in input 2")
legend('Input 1', 'Input 2', 'Tank1', 'Tank 2')
xlim([630 680])
ylim([-5 51])

%% PLot the results Input 2

indx = (time > 944.5);
figure;
plot(time(indx),Input1(indx), 'LineWidth', 1.5)
hold all
plot(time(indx),Input2(indx), 'LineWidth', 1.5)
plot(time(indx),Tank1(indx), 'LineWidth', 1.5)
plot(time(indx),Tank2(indx), 'LineWidth', 1.5)
grid on
title("Manual Control")
legend('Input 1', 'Input 2', 'Tank1', 'Tank 2')
xlim([944.5 1035])
ylim([-2 62])

%% Non-minimum phase case
fig = gcf;
dataObjs = findobj(fig,'-property','YData');
Input1 = dataObjs(1).YData;
Input2 = dataObjs(2).YData;
Tank1 = dataObjs(7).YData;
Tank2 = dataObjs(8).YData;
dataObjs = findobj(fig,'-property','xData');
time = dataObjs(1).XData;

%%
% PLot the results Input 2
indx = (time > 1587) & (time < 1653);
figure;
hold all
plot(time(indx),Input2(indx), 'LineWidth', 1.5)
plot(time(indx),Tank1(indx), 'LineWidth', 1.5)
plot(time(indx),Tank2(indx), 'LineWidth', 1.5)
grid on
title("Step response in input 2")
legend('Input 2', 'Tank1', 'Tank 2')
%%
indx2 =(time > 1653.6);
figure;
plot(time(indx2),Input1(indx2), 'LineWidth', 1.5)
hold all
plot(time(indx2),Tank1(indx2), 'LineWidth', 1.5)
plot(time(indx2),Tank2(indx2), 'LineWidth', 1.5)
grid on
title("Step response in input 1")
legend('Input 1', 'Tank1', 'Tank 2')
%%
figure
subplot(2,1,1)
plot(time(indx2),Input1(indx2), 'LineWidth', 1.5)
hold all
plot(time(indx2),Tank1(indx2), 'LineWidth', 1.5)
plot(time(indx2),Tank2(indx2), 'LineWidth', 1.5)
grid on
title("Step response in input 1")
legend('Input 1', 'Tank1', 'Tank 2')
subplot(2,1,2)
hold all
plot(time(indx),Input2(indx), 'LineWidth', 1.5)
plot(time(indx),Tank1(indx), 'LineWidth', 1.5)
plot(time(indx),Tank2(indx), 'LineWidth', 1.5)
grid on
title("Step response in input 2")
legend('Input 2', 'Tank1', 'Tank 2')


%% Lab Sess 2: Minimum phase case nominal controller
%% Obtain data from figure
fig = gcf;
dataObjs = findobj(fig,'-property','YData');
Ref1 = dataObjs(3).YData;
Ref2 =  dataObjs(4).YData;
Tank1 = dataObjs(8).YData;
Tank2 = dataObjs(7).YData;
dataObjs = findobj(fig,'-property','xData');
time = dataObjs(1).XData;


fig = gcf;
dataObjs = findobj(fig,'-property','YData');
dist_Input1 = dataObjs(1).YData;
dist_Input2 = dataObjs(2).YData;
dist_Ref1 = dataObjs(3).YData;
dist_Ref2 =  dataObjs(4).YData;
dist_Tank1 = dataObjs(8).YData;
dist_Tank2 = dataObjs(7).YData;
dataObjs = findobj(fig,'-property','xData');
dist_time = dataObjs(1).XData;


%% PLot the results Input 1
indx = (time > 2060) & (time < 2090);
figure;
plot(time(indx),Input1(indx), 'LineWidth', 1.5)
hold all
plot(time(indx),Input2(indx), 'LineWidth', 1.5)
plot(time(indx),Tank1(indx), 'LineWidth', 1.5)
plot(time(indx),Tank2(indx), 'LineWidth', 1.5)
plot(time(indx),Ref1(indx), 'LineWidth', 1.5)
plot(time(indx),Ref2(indx), 'LineWidth', 1.5)
grid on
title("Step response in input 1")
legend('Input 1', 'Input 2', 'Tank1', 'Tank 2', 'Ref1', 'Ref2')

%% Report Figure
figure;
subplot(1,2,1)
indx = (time > 2142) & (time < 2175);
plot(time(indx),Tank1(indx), 'LineWidth', 1.5)
hold all
plot(time(indx),Tank2(indx), 'LineWidth', 1.5)
plot(time(indx),Ref1(indx), 'LineWidth', 1.5)
plot(time(indx),Ref2(indx), 'LineWidth', 1.5)
grid on
title("Step response")
legend('Tank1', 'Tank 2', 'Ref1', 'Ref2')
subplot(1,2,2)
dist_indx = (time > 2060) & (time < 2090);
plot(time(dist_indx),Tank1(dist_indx), 'LineWidth', 1.5)
hold all
plot(time(dist_indx),Tank2(dist_indx), 'LineWidth', 1.5)
plot(time(dist_indx),Ref1(dist_indx), 'LineWidth', 1.5)
plot(time(dist_indx),Ref2(dist_indx), 'LineWidth', 1.5)
grid on
title("Disturbance Rejection")
legend('Tank1', 'Tank 2', 'Ref1', 'Ref2')


%%
% Rise time 6 sec
% Overshoot 0
% Disturbance 15 sec
