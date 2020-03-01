classdef Trim < AE5224.quad_rotor.control.Controller
    %TRIM Quadrotor trim tracking controller
    
    properties (Access = protected)
        p_e_ref_t;  % Position traj [m]
        a_e_ref_t;  % Accel traj [m]
        th_z_ref_t; % Heading traj [rad]
    end
    
    methods (Access = public)
        function obj = Trim(model, trim, del_t)
            %obj = TRIM(model, x_st)
            %   Construct trim tracking controller
            %   - model = Quadrotor model [AE5224.quad_rotor.Model]
            % 	- trim = Trim conditions [AE5224.Trim]
            %   - del_t = Time delta [s]
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            import('AE5224.quad_rotor.trim.solve');
            
            % Superconstructor
            obj@AE5224.quad_rotor.control.Controller(model, del_t);
            
            % Trim trajectory functions
            [x_st, ~] = solve(model, trim);
            [p_e, ~, v_e, ~] = unpack_x(x_st);
            w_ez = trim.w_e(3);
            if isinf(trim.R)
                obj.p_e_ref_t = @(t) p_e + v_e * t;
                obj.a_e_ref_t = @(t) zeros(3, 1);
            else
                obj.p_e_ref_t = @(t) [
                    trim.R * sin(w_ez * t);
                    trim.R * (1 - cos(w_ez * t));
                    p_e(3) + v_e(3) * t];
                obj.a_e_ref_t = @(t) [
                    trim.R * w_ez^2 * -sin(w_ez * t);
                    trim.R * w_ez^2 * +cos(w_ez * t);
                    0];
            end
            obj.th_z_ref_t = @(t) w_ez * t;
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x, t)
            %u = UPDATE(obj, x, t)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - t = Current time [s]
            %   - u = Unsurated control vector
            p_e_ref = obj.p_e_ref_t(t);
            a_e_ref = obj.a_e_ref_t(t);
            th_z_ref = obj.th_z_ref_t(t);
            u = update_@AE5224.quad_rotor.control.Controller(...
                obj, x, p_e_ref, a_e_ref, th_z_ref);
        end
    end
end