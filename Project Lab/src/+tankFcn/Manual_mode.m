function Manual_mode(app)
    %% CONTROLLER SWITCH
    app.controller=app.Manual;
    app.system.ref=NaN;
    
    %% CONTROLLER SETTING PANAL
    app.RefSlider_1.Enable='Off';
    app.RefEditField_1.Enable='Off';
    app.RefSlider_2.Enable='Off';
    app.RefEditField_2.Enable='Off';
    
    app.InputSlider_1.Enable='On';
    app.InputEditField_1.Enable='On';
    app.InputSlider_2.Enable='On';
    app.InputEditField_2.Enable='On';
    
    app.LoadFeedbackControllerButton.Enable='Off';
end