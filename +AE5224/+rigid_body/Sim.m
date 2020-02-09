classdef Sim < handle
    %SIM Class for 3D rigid body simulation
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        body;   % Rigid body model [AE5224.rigid_body.Body]
        t;      % Current time [s]
        del_t;  % Simulation time delta [s]
        p_e;    % Position [Earth, m]
        q_e;    % Attitude [Earth, quaternion]
        v_e;    % Linear velocity [Earth, m]
        w_b;    % Angular velocity [Body, rad/s]
    end
    
    methods (Access = public)
        function obj = Sim(body, del_t, p_e, q_e, v_e, w_b)
            %obj = SIM(body, del_t, p_e, q_e, v_e, w_b)
            %   Construct rigid body simulator
            %   
            %   Inputs:
            %   - body = Rigid body model [AE5224.rigid_body.Body]
            %   - del_t = Simulation time delta [s]
            %   - p_e = Position [Earth, m]
            %   - q_e = Attitude [Earth, quaternion]
            %   - v_e = Linear velocity [Earth, m]
            %   - w_b = Angular velocity [Body, rad/s]
            %   
            %   Default initial conditions are zero
            
            % Default args
            if nargin < 3, p_e = [0; 0; 0]; end
            if nargin < 4, q_e = [1; 0; 0; 0]; end
            if nargin < 5, v_e = [0; 0; 0]; end
            if nargin < 6, w_b = [0; 0; 0]; end
            
            % Construction
            obj.body = body;
            obj.t = 0;
            obj.del_t = del_t;
            obj.p_e = p_e;
            obj.q_e = q_e;
            obj.v_e = v_e;
            obj.w_b = w_b;
        end
        
        function update(obj, u)
            %UPDATE(obj, F_b, M_b)
            %   Run one simulation loop
            %   
            %   Inputs:
            %   - u = Input vector
            
            % Imports
            import('AE5224.rigid_body.Body.pack');
            import('AE5224.rigid_body.Body.unpack');
            import('runge_kutta.rk4');
            
            % RK4 integration
            x0 = pack(obj.p_e, obj.q_e, obj.v_e, obj.w_b);
            dx_dt = @(t, x) obj.body.dynamics(x, u);
            x1 = rk4(dx_dt, 0, x0, obj.del_t);
            [obj.p_e, obj.q_e, obj.v_e, obj.w_b] = unpack(x1);
            obj.t = obj.t + obj.del_t;
            
            % Quaternion normalization
            obj.q_e = obj.q_e / norm(obj.q_e);
        end
    end
end