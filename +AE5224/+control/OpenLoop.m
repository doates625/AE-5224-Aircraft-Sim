classdef OpenLoop < AE5224.control.Controller
    %OPENLOOP Class for open-loop UAV control
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        u_st;   % Trim control vector
    end
    
    methods (Access = public)
        function obj = OpenLoop(model, u_st)
            %obj = OPENLOOP(model, u_st)
            %   Construct open-loop controller
            %   - model = Aircraft model [AE5224.Model]
            %   - u_st = Trim control vector
            obj@AE5224.control.Controller(model);
            obj.u_st = u_st;
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, ~, ~)
            %u = UPDATE_(obj, x, t)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - t = Current time [s]
            %   - u = Unsaturated control vector
            u = obj.u_st;
        end
    end
end