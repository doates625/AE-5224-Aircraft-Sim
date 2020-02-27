classdef (Abstract) Controller
    %CONTROLLER Superclass for UAV control systems
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        model;  % Model [AE5224.Model]
    end
    
    methods (Access = public)
        function obj = Controller(model)
            %obj = CONTROLLER(model)
            %   Construct controller
            %   - model = Aircraft model [AE5224.Model]
            obj.model = model;
        end
        
        function u = update(obj, x, t)
            %u = UPDATE(obj, x)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - t = Current time [s]
            %   - u = Saturated control vector
            import('controls.clamp');
            u = obj.update_(x, t);
            u = clamp(u, obj.model.u_min, obj.model.u_max);
        end
    end
    
    methods (Access = protected, Abstract)
        u = update_(obj, x, t)
        %u = UPDATE(obj, x, t)
        %   Update control output with new state
        %   - x = State [p_e; q_e; v_e; w_b]4
        %   - t = Current time [s]
        %   - u = Unsurated control vector
    end
end