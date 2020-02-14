classdef OpenLoop < AE5224.control.Controller
    %OPENLOOP Class for open-loop UAV control
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        u_st;   % Trim control vector
    end
    
    methods (Access = public)
        function obj = OpenLoop(u_st)
            %obj = OPENLOOP(u_st)
            %   Construct open-loop controller
            %   - u_st = Trim control vector
            obj.u_st = u_st;
        end
        
        function u = update(obj, ~)
            %u = UPDATE(obj, x)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - u = Control vector
            u = obj.u_st;
        end
    end
end