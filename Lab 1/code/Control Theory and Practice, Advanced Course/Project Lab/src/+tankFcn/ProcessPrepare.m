function ProcessPrepare(app)
% --- INITIALIZE PARAMETERS --- %
tankFcn.FreshPara;
%% --- CONSTRUCTOR: SYSTEM --- %
if tankFcn.HW_detect()
    app.HW=System(sysInfo,[app.RefSlider_1.Value, app.RefSlider_2.Value]);
end
app.SW=Model(simInfo,[app.RefSlider_1.Value, app.RefSlider_2.Value]);
switch app.SimRealSwitch.Value
    case 'Real time'
        app.system = app.HW;
    case 'Simulator'
        app.system = app.SW;
end

%% --- CONSTRUCTOR: CONTROLLER --- %
%     FeedbackName = app.FeedbackEditField.Value;
app.Feedback=ControllerFeedback([],Ts,max_vol);
app.Manual=ControllerManual(max_vol);

switch app.ManualFeedbackSwitch.Value
    case 'Manual'
        tankFcn.Manual_mode(app);
    case 'Feedback'
        tankFcn.Feedback_mode(app);
end

%% --- SPECIFICATION: plotting --- %
app.figureOption = figureOption;
tankFcn.PlotPrepare(app);

%% --- CONSTRUCTOR: TIMER --- %
app.Timer = timer;
% --- TIMER SPECIFICATION --- %
app.Timer.ExecutionMode = 'fixedRate';
if isa(app.system,'Model')
    switch app.SimulatorSpeedUpDropDown.Value
        case '1x'; scale = 5;
        case '2x'; scale = 4;
        case '3x'; scale = 3;
        case '4x'; scale = 2;
        case '5x'; scale = 1;
    end
    app.Timer.Period = app.system.Ts*(scale/5);
else
    app.Timer.Period = app.system.Ts;
    app.SimulatorSpeedUpDropDown.Value = '1x';
    app.SimulatorSpeedUpDropDown.Enable = 'Off';
    app.SimulatorspeedupLabel.Enable = 'Off';
end
app.Timer.TimerFcn = {@tankFcn.mainLoop,app}; % MAIN LOOP EXECUTED BY TIMER

end



