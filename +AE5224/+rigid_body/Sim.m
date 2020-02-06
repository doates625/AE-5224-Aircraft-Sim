classdef Sim < handle
    %SIM Class for 3D rigid body simulation
    %   
    %   For this simulator, Earth is assumed to be an inertia reference frame.
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        body;   % Rigid body model [AE5224.rigid_body.Body]
        t;      % Current time [s]
        p_e;    % Position [Earth, m]
        q_e;    % Attitude [Earth, quaternion]
        v_e;    % Linear velocity [Earth, m]
        w_b;    % Angular velocity [Body, rad/s]
        del_t;  % Simulation time delta [s]
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
            obj.p_e = p_e;
            obj.q_e = q_e;
            obj.v_e = v_e;
            obj.w_b = w_b;
            obj.del_t = del_t;
        end
        
        function update(obj, F_b, M_b)
            %UPDATE(obj, F_b, M_b)
            %   Run one simulation loop
            %   
            %   Inputs:
            %   - F_b = Net forces [Body, N]
            %   - M_b = Net moments [Body, N*m]
            
            % Imports
            import('runge_kutta.rk4');
            
            % RK4 integration
            s0 = obj.pack(obj.p_e, obj.q_e, obj.v_e, obj.w_b);
            ds_dt = @(t, s) obj.derivative(s, F_b, M_b);
            s1 = rk4(ds_dt, 0, s0, obj.del_t);
            [obj.p_e, obj.q_e, obj.v_e, obj.w_b] = obj.unpack(s1);
            obj.t = obj.t + obj.del_t;
            
            % Quaternion normalization
            obj.q_e = obj.q_e / norm(obj.q_e);
        end
    end
    
    methods (Access = protected)
        function ds_dt = derivative(obj, s, F_b, M_b)
            %ds_dt = DERIVATIVE(obj, s, F_b, M_b)
            %   Compute state time-derivative
            %   
            %   Inputs:
            %   - s = Combined state vector [p_e; q_e; v_e; w_b]
            %   - F_b = Net forces Body-frame [N]
            %   - M_b = Net moments Body-frame [N*m]
            %   
            %   Outputs:
            %   - ds_dt = State time-derivative
            
            % Imports
            import('quat.Quat');
            
            % Unpack states
            [~, q_e_, v_e_, w_b_] = obj.unpack(s);
            
            % Position and attitude
            p_e_dot = v_e_;
            q_e_dot = Quat(q_e_) * Quat([0; 0.5*w_b_]);
            q_e_dot = q_e_dot.vector();
            
            % Linear velocity
            q_e_ = Quat(q_e_);
            F_e = q_e_.rotate(F_b);
            v_e_dot = F_e / obj.body.m;
            
            % Angular velocity
            L_b = obj.body.I_b * w_b_;
            L_b_dot = M_b - cross(w_b_, L_b);
            w_b_dot = obj.body.I_b \ L_b_dot;
            
            % Pack states
            ds_dt = obj.pack(p_e_dot, q_e_dot, v_e_dot, w_b_dot);
        end
    end
    
    methods (Access = protected, Static)
        function s = pack(p_e, q_e, v_e, w_b)
            %s = PACK(p_e, q_e, v_e, w_b)
            %   Combines states into single vector
            s = [p_e; q_e; v_e; w_b];
        end
        
        function [p_e, q_e, v_e, w_b] = unpack(s)
            %[p_e, q_e, v_e, w_b] = UNPACK(s)
            %   Extracts states from single vector
            p_e = s(1:3);
            q_e = s(4:7);
            v_e = s(8:10);
            w_b = s(11:13);
        end
    end
end