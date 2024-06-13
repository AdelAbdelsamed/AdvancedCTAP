function ProcessOverflow(app)
    %% tank1 overflow checking
    if app.system.History(end,1)>=100 && app.system.History(end-1,1)<=100
        app.tank1Overflow.Color = [1,0,0];
    end
    if app.system.History(end,1)<=100 && app.system.History(end-1,1)>=100
        app.tank1Overflow.Color = [0.65,0.65,0.65];
    end
    
    %% tank2 overflow checking
    if app.system.History(end,2)>=100 && app.system.History(end-1,2)<=100
        app.tank2Overflow.Color = [1,0,0];
    end
    if app.system.History(end,2)<=100 && app.system.History(end-1,2)>=100
        app.tank2Overflow.Color = [0.65,0.65,0.65];
    end
    
    %% tank3 overflow checking
    if app.system.History(end,3)>=100 && app.system.History(end-1,3)<=100
        app.tank3Overflow.Color = [1,0,0];
    end
    if app.system.History(end,3)<=100 && app.system.History(end-1,3)>=100
        app.tank3Overflow.Color = [0.65,0.65,0.65];
    end
    
    %% tank4 overflow checking
    if app.system.History(end,4)>=100 && app.system.History(end-1,4)<=100
        app.tank4Overflow.Color = [1,0,0];
    end
    if app.system.History(end,4)<=100 && app.system.History(end-1,4)>=100
        app.tank4Overflow.Color = [0.65,0.65,0.65];
    end
end