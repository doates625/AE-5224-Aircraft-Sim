classdef Sim < handle
    %SIM Class for 3D rigid body simulation
    %   
    %   State vecor x:
    %   - p_e = Position Earth [m]
    %   - q_e = Attitude Earth [quat]
    %   - v_e = Velocity Earth [m/s]
    %   - w_b = Angle rate Body [rad/s]
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        body;   % Rigid body model [AE5224.rigid_body.Body]
        t;      % Current time [s]
        x;      % State vector
        del_t;  % Simulation time delta [s]
    end
    
    methods (Access = public)
        function obj = Sim(body, x, del_t)
            %obj = SIM(body, x, del_t)
            %   Construct rigid body simulator
            %   
            %   Inputs:
            %   - body = Rigid body model [AE5224.rigid_body.Body]
            %   - x = Initial state [p_e; q_e; v_e; w_b]
            %   - del_t = Simulation time delta [s]
            obj.body = body;
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
            import('AE5224.rigid_body.Body.unpack');
            import('AE5224.rigid_body.Body.pack');
            import('runge_kutta.rk4');
            
            % RK4 integration
            dx_dt = @(t, x) obj.body.dynamics(x, u);
            obj.x = rk4(dx_dt, 0, obj.x, obj.del_t);
            obj.t = obj.t + obj.del_t;
            
            % Quaternion normalization
            [p_e, q_e, v_e, w_b] = unpack(obj.x);
            q_e = q_e / norm(q_e);
            obj.x = pack(p_e, q_e, v_e, w_b);
        end
    end
end