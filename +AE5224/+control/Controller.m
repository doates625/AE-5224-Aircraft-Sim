classdef (Abstract) Controller
    %CONTROLLER Superclass for UAV control systems
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        u_min;  % Min controls vector
        u_max;  % Max controls vector
    end
    
    methods (Access = public)
        function obj = Controller(u_min, u_max)
            %obj = CONTROLLER(u_min, u_max)
            %   Construct controller
            %   - u_min = Min controls vector
            %   - u_max = Max controls vector
            obj.u_min = u_min;
            obj.u_max = u_max;
        end
        
        function u = update(obj, x)
            %u = UPDATE(obj, x)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - u = Saturated control vector
            u = obj.update_(x);
            u = min(max(obj.u_min, u), obj.u_max);
        end
    end
    
    methods (Access = protected, Abstract)
        u = update_(obj, x)
        %u = UPDATE(obj, x)
        %   Update control output with new state
        %   - x = State [p_e; q_e; v_e; w_b]
        %   - u = Unsurated control vector
    end
end