%% --- Minphase --- %
% Operating actuator proportional constants
k1 = 4.32;     % i cm^3/(Vs)
k2 = 3.74;     % i cm^3/(Vs)
% Outlet areas
a1 = 0.1678;   % i cm^2
a2 = 0.1542;   % i cm^2
a3 = 0.06743;  % i cm^2
a4 = 0.06504;  % i cm^2
% Valve settings
gam1 = 0.625;
gam2 = 0.625;

% Linearization point
% h10 = 18; = 72%     % i cm (nedre vänstra tanken)
% h20 = 18; = 72%     % i cm (nedre högra tanken)
% h30 = 14; = 56%     % i cm (övre vänstra tanken)
% h40 = 19; = 76%     % i cm (övre högra tanken)

%% --- Nonminphase --- %
% % Operating actuator proportional constants
% k1 = x;     % i cm^3/(Vs)
% k2 = x;     % i cm^3/(Vs)
% % Outlet areas
% a1 = x;   % i cm^2
% a2 = x;   % i cm^2
% a3 = x;  % i cm^2
% a4 = x;  % i cm^2
% % Valve settings
% gam1 = x;
% gam2 = x;

% h10 = x;     % i cm (nedre vänstra tanken)
% h20 = x;     % i cm (nedre högra tanken)
% h30 = x;     % i cm (övre vänstra tanken)
% h40 = x;     % i cm (övre högra tanken)

%% --- Common parameters --- %
% Cross section areas
A1 = 15.5179;   % cm^2
A2 = 15.5179;   % cm^2
A3 = 15.5179;   % cm^2
A4 = 15.5179;   % cm^2
% Sensor proportional constant
kc = 0.2;       % V/cm