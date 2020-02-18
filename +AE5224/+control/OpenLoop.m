classdef OpenLoop < AE5224.control.Controller
    %OPENLOOP Class for open-loop UAV control
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        u_st;   % Trim control vector
    end
    
    methods (Access = public)
        function obj = OpenLoop(u_st, u_min, u_max)
            %obj = OPENLOOP(u_st)
            %   Construct open-loop controller
            %   - u_st = Trim control vector
            %   - u_min = Min controls vector
            %   - u_max = Max controls vector
            
            % Default args
            if nargin < 2, u_min = -inf(size(u_st)); end
            if nargin < 3, u_max = +inf(size(u_st)); end
            
            % Construction
            obj@AE5224.control.Controller(u_min, u_max);
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