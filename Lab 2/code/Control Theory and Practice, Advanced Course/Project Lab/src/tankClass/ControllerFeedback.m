% Controller class: class for hardware
% --- PROPERTY --- %
% Controller.I_state:              history error integration term                                   [1x1]
% Controller.D_state:              last sampling storing                                            [1x1]
% Controller.controlSignal_before: for AntiWindUp, control signal before control signal saturation  [1x1]
% Controller.controlSignal_after:  for AntiWindUp, control signal after control signal saturation   [1x1]
% Controller.controlSignal_max:    for AntiWindUp, control signal saturation value                  [1x1]
% Controller.AntiWindUp_flag:      for AntiWindUp, activation flag                                  [1x1]
% Controller.K:   controller parameter                                             [1x1]
% Controller.Ti:  controller parameter                                             [1x1]
% Controller.Td:  controller parameter                                             [1x1]
% Controller.N:   low pass filter parameter for controller derivative part
% Controller.Ts:  controller sampling time                                         [1x1]
% --- METHOD --- %
% Controller.compute(): compute control signal and update I_state & D_state

classdef ControllerFeedback < handle
    properties
        controllerState
        controlSignal_before
        controlSignal_after
        controlSignal_max
        AntiWindUp_flag
        Ad
        Bd
        Cd
        Dd
        Ts
        controllerFound
        controllerCorrectDimensions
    end
    methods
        % --- CONSTRUCTOR --- %
        function obj=ControllerFeedback(name,Ts,u_max)
            if ~isa(name,'char')
                obj.Ad = zeros(2);
                obj.Bd = zeros(2);
                obj.Cd = zeros(2);
                obj.Dd = zeros(2);
                obj.controllerFound = false;
                obj.controllerCorrectDimensions = false;
            else
                try
                    F = load(name);
                    try
                        F = ss(F.A,F.B,F.C,F.D);
                        F = c2d(F,Ts);
                        obj.Ad = F.A;
                        obj.Bd = F.B;
                        obj.Cd = F.C;
                        obj.Dd = F.D;
                        obj.controllerFound = true;
                        obj.controllerCorrectDimensions = true;
                    catch                      
                        obj.controllerCorrectDimensions = false;
                    end
                    
                catch                   
                    obj.controllerFound = false;
                end
            end
            obj.controllerState = zeros(size(obj.Ad,1),1);
            obj.controlSignal_before=[0, 0];
            obj.controlSignal_after=[0,0];
            obj.controlSignal_max=u_max;
            obj.Ts=Ts;
            
        end
        
        % --- METHOD: COMPUTING --- %
        function u_after = compute(obj,sample,ref)
            
            % --- ERROR CALCULATION --- %
            e = (ref-sample)'; % [2x1]
%             e = e*25/100; % cm;
            
            % --- CONTROLLER STATES --- %
            obj.controllerState = obj.Ad*obj.controllerState + obj.Bd*e;
            
            % --- INPUT SATURATION --- %
            u_before = obj.Cd*obj.controllerState + obj.Dd*e; %[2x1]
            % scale back to voltage and transpose
            u_before = (obj.controlSignal_max/100)*u_before';  %[1x2]
            u_after = NaN(1,2);
            for i = 1:2
                u_after(1,i) = min(obj.controlSignal_max,max(u_before(1,i),0)); % input voltage saturation
            end
            % --- CONTROLLER UPDATING --- %
            obj.controlSignal_before = u_before; % (Not used)
            obj.controlSignal_after = u_after;
        end
    end
end
