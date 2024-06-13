function PlotUpdate(app)
    %% PREPARE
    NumSampleToPlot = app.figureOption.NumSampleToPlot;
    
    %% DATA PLOT
    % TANK LEVEL
    if app.tank1CheckBox.Value
        set(app.tank1_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,1));
    end
    if app.tank2CheckBox.Value
        set(app.tank2_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,2));
    end
    if app.tank3CheckBox.Value
        set(app.tank3_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,3));
    end
    if app.tank4CheckBox.Value
        set(app.tank4_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,4));
    end
    % REFERENCE
    if app.Ref1CheckBox.Value
        set(app.ref1_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,5));
    end
    if app.Ref2CheckBox.Value
        set(app.ref2_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,6));
    end
    % CONTROL SIGNAL
    if app.ControlSignal1CheckBox.Value
        set(app.controlSignal1_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,7));
    end
    if app.ControlSignal2CheckBox.Value
        set(app.controlSignal2_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,8));
    end
%     if app.PpartCheckBox.Value
%         set(app.Ppart_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,8));
%     end
%     if app.IpartCheckBox.Value
%         set(app.Ipart_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,9));
%     end
%     if app.DpartCheckBox.Value
%         set(app.Dpart_plot,'YData',app.system.History(end-NumSampleToPlot+1:end,10));
%     end
    %% GAUGE PLOT
    app.tank1Gauge.Value = app.system.state(1,1);
    app.tank2Gauge.Value = app.system.state(1,2);
    if isa(app.system,'Model')
        app.tank3Gauge.Value = app.system.state(1,3);
        app.tank4Gauge.Value = app.system.state(1,4);  
    end
    
end