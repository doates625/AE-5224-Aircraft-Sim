classdef (Abstract) Controller
    %CONTROLLER Superclass for UAV control systems
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public, Abstract)
        u = update(obj, x)
        %u = UPDATE(obj, x)
        %   Update control output with new state
        %   - x = State [p_e; q_e; v_e; w_b]
        %   - u = Control vector
    end
end