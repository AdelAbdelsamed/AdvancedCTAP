% --- SETTING: BASIC --- %
Ts=0.05;
max_vol = 3; % Outputvoltage to old power module: 3v, new power module: 5v
% The module then powers a 15? V pump.
max_vol_in = 5; % Measured from SCM-68 at 100%
time=1800;  % 30 min to store
NumSampleToHistory=int32(time/Ts);
NumSampleToPlot=int32(app.TspanEditField.Value/Ts);

% --- SETTING: GUI PLOT --- %
f1='axis_lim'; 
v1=[-app.TspanEditField.Value,0,app.YminEditField.Value,app.YmaxEditField.Value];
f2='NumSampleToHistory'; 
v2=NumSampleToHistory;
f3='NumSampleToPlot'; 
v3=NumSampleToPlot;
figureOption = struct(f1,v1,f2,v2,f3,v3);

% --- SETTING: REALTIME --- %
sysInfo = struct('Ts',Ts,f2,v2,'max_vol_in',max_vol_in,'max_vol',max_vol);

% --- SETTING: SIMULATION --- %
SimPara;

model.alpha1=a1/A1;
model.alpha2=a2/A2;
model.alpha3=a3/A3;
model.alpha4=a4/A4;
model.alpha13=a3/A1;
model.alpha24=a4/A2;
model.beta1=gam1*k1/A1*15/max_vol; %cm/Vs (power module outputs 0-15 V from 0-3 V input)
model.beta2=gam2*k2/A2*15/max_vol;
model.beta32=(1-gam2)*k2/A3*15/max_vol;
model.beta41=(1-gam1)*k1/A4*15/max_vol;
delay = 0; % second
simInfo = struct('Ts',Ts,f2,v2,'model',model,'delay',delay,'max_vol',max_vol);

