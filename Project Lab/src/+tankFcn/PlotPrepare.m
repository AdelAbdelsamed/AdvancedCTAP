function PlotPrepare(app)
    % --- PLOT PREPARATION --- %
    NumToPlot = app.figureOption.NumSampleToPlot;
    t = linspace(app.figureOption.axis_lim(1,1),app.figureOption.axis_lim(1,2),NumToPlot);
    grid(app.DataAquisition,'on');
    cla(app.DataAquisition)

    %% PLOT OBJECT
    hold(app.DataAquisition,'on')
    % --- COLOR SPECIFICATION SECTION --- % 
    tank1Color  = [ 0         0.4470    0.7410 ];
    tank2Color  = [ 0         0.4470    0.7410 ];
    tank3Color  = [ 0.8500    0.3250    0.0980 ];
    tank4Color  = [ 0.8500    0.3250    0.0980 ];
    ref1Color   = [ 0.9290    0.6940    0.1250 ];
    ref2Color   = [ 0.9290    0.6940    0.1250 ];
    input1Color = [ 0.4940    0.1840    0.5560 ];
    input2Color = [ 0.4940    0.1840    0.5560 ];

    % TANK LEVEL
    app.tank1_plot = plot(app.DataAquisition,t,NaN(NumToPlot,1),'Color',tank1Color);
    app.tank2_plot = plot(app.DataAquisition,t,NaN(NumToPlot,1),'Color',tank2Color,'LineStyle','-.');
    app.tank3_plot = plot(app.DataAquisition,t,NaN(NumToPlot,1),'Color',tank3Color);
    app.tank4_plot = plot(app.DataAquisition,t,NaN(NumToPlot,1),'Color',tank4Color,'LineStyle','-.');
    % REFERENCE
    app.ref1_plot = plot(app.DataAquisition,t,NaN(NumToPlot,1),'Color',ref1Color);
    app.ref2_plot = plot(app.DataAquisition,t,NaN(NumToPlot,1),'Color',ref2Color,'LineStyle','-.');
    % CONTROL SIGNAL
    app.controlSignal1_plot = plot(app.DataAquisition,t,NaN(NumToPlot,1),'Color',input1Color);
    app.controlSignal2_plot = plot(app.DataAquisition,t,NaN(NumToPlot,1),'Color',input2Color,'LineStyle','-.');
    hold(app.DataAquisition,'off')
    legend(app.DataAquisition,'Tank1','Tank2','Tank3','Tank4','Reference1','Reference2','Input1','Input2','Location','northwest');
    
    %% AXIS LIMIT
    app.DataAquisition.XLim=app.figureOption.axis_lim(1:2);
    app.DataAquisition.YLim=app.figureOption.axis_lim(3:4);
    
    %% YAXIS LOCATION
    app.DataAquisition.YAxisLocation = 'right';
end