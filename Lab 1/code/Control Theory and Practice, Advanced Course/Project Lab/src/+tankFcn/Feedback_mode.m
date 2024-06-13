function Feedback_mode(app)
    %% CONTROLLER SWITCH
    app.controller=app.Feedback;
    app.system.ref(1,1)=app.RefSlider_1.Value;
    app.system.ref(1,2)=app.RefSlider_2.Value;
    
    %% CONTROLLER SETTING PANAL
    app.RefSlider_1.Enable='On';
    app.RefEditField_1.Enable='On';
    app.Ref1CheckBox.Value = true;
    app.InputSlider_1.Enable='Off';
    app.InputEditField_1.Enable='Off';
    
    app.RefSlider_2.Enable='On';
    app.RefEditField_2.Enable='On';
    app.Ref2CheckBox.Value = true;
    app.InputSlider_2.Enable='Off';
    app.InputEditField_2.Enable='Off';
    
%     app.FeedbackEditField.Enable='On';  
    app.LoadFeedbackControllerButton.Enable='On';
end