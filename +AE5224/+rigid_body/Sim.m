classdef Sim < handle
    %SIM Class for 3D rigid body simulation
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        model;  % Rigid body model [AE5224.rigid_body.Model]
        t;      % Current time [s]
        x;      % State vector
        del_t;  % Simulation time delta [s]
    end
    
    methods (Access = public)
        function obj = Sim(model, x, del_t)
            %obj = SIM(model, x, del_t)
            %   Construct rigid body simulator
            %   
            %   Inputs:
            %   - model = Rigid body model [AE5224.rigid_body.Model]
            %   - x = Initial state [p_e; q_e; v_e; w_b]
            %   - del_t = Simulation time delta [s]
            obj.model = model;
            obj.t = 0;
            obj.del_t = del_t;
            obj.x = x;
        end
        
        function update(obj, u)
            %UPDATE(obj, F_b, M_b)
            %   Run one simulation loop
            %   
            %   Inputs:
            %   - u = Input vector
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            import('AE5224.rigid_body.Model.pack_x');
            import('runge_kutta.rk4');
            
            % RK4 integration
            dx_dt = @(t, x) obj.model.dynamics(x, u);
            obj.x = rk4(dx_dt, 0, obj.x, obj.del_t);
            obj.t = obj.t + obj.del_t;
            
            % Quaternion normalization
            [p_e, q_e, v_e, w_b] = unpack_x(obj.x);
            q_e = q_e / norm(q_e);
            obj.x = pack_x(p_e, q_e, v_e, w_b);
        end
    end
end