% Controller class: class for hardware
% --- PROPERTY --- %
% Controller.controlSignal_before: control signal before control signal saturation  [1x1]
% Controller.controlSignal_after:  control signal after control signal saturation   [1x1]
% Controller.controlSignal_max:    control signal saturation value                  [1x1]
% --- METHOD --- %
% Controller.compute(): compute control signal, 'sample' is a dummy input
% for consistance with class: ControllerPID

classdef ControllerManual < handle
   
   properties
       controlSignal_before
       controlSignal_after
       controlSignal_max
       numberOfControllerStates
   end
   
   methods
       % --- CONSTRUCTOR --- %
       function obj=ControllerManual(u_max)
           obj.controlSignal_before=[0,0];
           obj.controlSignal_after=[0,0];
           obj.controlSignal_max=u_max;
           obj.numberOfControllerStates=0;
       end
       
       % --- METHOD: COMPUTING --- %
       function u_after = compute(obj,~,~)
           u_before = NaN(1,2);
           u_after = NaN(1,2);
           for i = 1:2
           u_before(1,i) = (obj.controlSignal_max/100)*obj.controlSignal_before(1,i); % scale back to voltage
           u_after(1,i) = min(obj.controlSignal_max,max(u_before(1,i),0)); % input voltage saturation
           % --- CONTROLLER UPDATING --- %
           obj.controlSignal_after(1,i) = u_after(1,i);
           end             
       end
   end
   
end
