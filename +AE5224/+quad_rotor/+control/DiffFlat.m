classdef DiffFlat < AE5224.control.Controller
    %DIFFFLAT Quadrotor differential flatness controller
    
    properties (SetAccess = protected)
        model;  % Quadrotor model [AE5224.quad_rotor.Model]
        traj;   % Trajectory plan [AE5224.quad_rotor.control.Traj]
    end
    
    methods (Access = public)
        function obj = DiffFlat(model, traj)
            %obj = DIFFFLAT(model, traj)
            %   Create differential flatness controller
            %   - model = Quadrotor model [AE5224.quad_rotor.Model]
            %   - traj = Trajectory plan [AE5224.quad_rotor.control.Traj]
            obj@AE5224.control.Controller(model.u_min, model.u_max);
            obj.model = model;
            obj.traj = traj;
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x, t)
            %u = UPDATE(obj, x, t)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]4
            %   - t = Current time [s]
            %   - u = Unsurated controls [rpm]
            
            % TODO
            u = zeros(4, 1);
        end
    end
end