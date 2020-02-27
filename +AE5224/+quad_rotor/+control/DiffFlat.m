classdef DiffFlat < AE5224.control.Controller
    %DIFFFLAT Quadrotor differential flatness controller
    
    properties (SetAccess = protected)
        model;  % Quadrotor model [AE5224.quad_rotor.Model]
        traj;   % Trajectory plan [AE5224.quad_rotor.control.TrajPlan]
    end
    properties (Access = protected)
        del_t;  % Update time delta [s]
        v_prev; % Last frame velocity [m/s]
        a_prev; % Last frame accel [m/s^2]
        k_p;    % Position gain [(m/s^4)/(m)]
        k_v;    % Velocity gain [(m/s^4)/(m/s)]
        k_a;    % Accel gain [(m/s^4)/(m/s^2)]
        k_j;    % Jerk gain [(m/s^4)/(m/s^3)]
    end
    
    methods (Access = public)
        function obj = DiffFlat(model, pole, del_t, traj)
            %obj = DIFFFLAT(model, pole, del_t, traj)
            %   Create differential flatness controller
            %   - model = Quadrotor model [AE5224.quad_rotor.Model]
            %   - pole = Snap feedback pole [s^-1]
            %   - del_t = Update time delta [s]
            %   - traj = Trajectory plan [AE5224.quad_rotor.control.TrajPlan]
            obj@AE5224.control.Controller(model.u_min, model.u_max);
            obj.model = model;
            obj.traj = traj;
            obj.k_p = 1*pole^4;
            obj.k_v = 4*pole^3;
            obj.k_a = 6*pole^2;
            obj.k_j = 4*pole^1;
            obj.del_t = del_t;
            obj.v_prev = zeros(3, 1);
            obj.a_prev = zeros(3, 1);
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x, t)
            %u = UPDATE(obj, x, t)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - t = Current time [s]
            %   - u = Unsurated controls [rpm]
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            import('AE224.get_g');
            import('quat.Quat');
            
            % UAV State
            [p_e, q_e, v_e, w_b] = unpack_x(x);
            a_e = (v_e - obj.v_prev) / obj.del_t;
            j_e = (a_e - obj.a_prev) / obj.del_t;
            obj.v_prev = v_e;
            obj.a_prev = a_e;
         
            % Reference with snap feedback
            [p_er, v_er, a_er, j_er, s_er] = obj.traj.get(t);
            s_er = s_er + ...
                obj.k_p * (p_er - p_e) + ...
                obj.k_v * (v_er - v_e) + ...
                obj.k_a * (a_er - a_e) + ...
                obj.k_j * (jr - j_e);
            
            % Heading ref
            t_zr = atan2(v_er(2), v_er(1));
            
            % Attitude ref
            m = obj.model.m;
            g = get_g();
            F_e = m * ([0; 0; -g] + a_er);
            F_eh = F_e / norm(F_e);
            F_bh = [0; 0; -1];
            dot_1 = 1 + F_bh.'*F_eh;
            q_ft = Quat([dot_1; cross(F_bh, F_eh)] / sqrt(2*dot_1));
            q_tz = Quat([0; 0; 1], t_zr);
            q_er = q_ft * q_tz;
            
            % Continue... (EQ 28-ish)
            %THAT PAGE DOESNT USE DF - just go with messy Cowlagi method...
            
            % TODO
            u = zeros(4, 1);
        end
    end
end