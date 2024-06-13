function PlotCapture(app, savename)
    figure; hold on;
    NumToCapture = ceil(app.SecondsToCaptureEditField.Value/app.system.Ts);
    t_start = app.system.counter*app.system.Ts-app.SecondsToCaptureEditField.Value;
    t_end = app.system.counter*app.system.Ts;
    t = linspace(t_start,t_end,NumToCapture)';
    y1 = app.system.History(end-NumToCapture+1:end,1);
    y2 = app.system.History(end-NumToCapture+1:end,2);
    y3 = app.system.History(end-NumToCapture+1:end,3);
    y4 = app.system.History(end-NumToCapture+1:end,4);
    yref1 = app.system.History(end-NumToCapture+1:end,5);
    yref2 = app.system.History(end-NumToCapture+1:end,6);
    u1 = app.system.History(end-NumToCapture+1:end,7);
    u2 = app.system.History(end-NumToCapture+1:end,8);

    % --- COLOR SPECIFICATION SECTION --- % 
    tank1Color  = [ 0         0.4470    0.7410 ];
    tank2Color  = [ 0         0.4470    0.7410 ];
    tank3Color  = [ 0.8500    0.3250    0.0980 ];
    tank4Color  = [ 0.8500    0.3250    0.0980 ];
    ref1Color   = [ 0.9290    0.6940    0.1250 ];
    ref2Color   = [ 0.9290    0.6940    0.1250 ];
    input1Color = [ 0.4940    0.1840    0.5560 ];
    input2Color = [ 0.4940    0.1840    0.5560 ];

    
%     if app.tank1CheckBox.Value
        plot(t,y1,'Color',tank1Color);
%     end
%     if app.tank2CheckBox.Value
        plot(t,y2,'Color',tank2Color,'LineStyle','-.');
%     end
%     if app.tank3CheckBox.Value
        plot(t,y3,'Color',tank3Color);
%     end
%     if app.tank4CheckBox.Value
        plot(t,y4,'Color',tank4Color,'LineStyle','-.');
%     end
%     if app.Ref1CheckBox.Value
        plot(t,yref1,'Color',ref1Color);
%     end
%     if app.Ref2CheckBox.Value
        plot(t,yref2,'Color',ref2Color,'LineStyle','-.');
%     end
%     if app.ControlSignal1CheckBox.Value
        plot(t,u1,'Color',input1Color);
%     end
%     if app.ControlSignal2CheckBox.Value
        plot(t,u2,'Color',input2Color,'LineStyle','-.');
%     end

    axis([t_start,t_end,app.figureOption.axis_lim(3:4)])
    xlabel('time [s]'); ylabel('Y [%]'); 
    legend('Tank1','Tank2','Tank3','Tank4','Reference1','Reference2','Input1','Input2','Location','northwest');    
    
    disp(['Saving output as: ' savename])
    save(savename, 't', 'y1', 'y2', 'y3', 'y4', 'yref1', 'yref2', 'u1', 'u2')    
   
    grid on;
    hold off;
end