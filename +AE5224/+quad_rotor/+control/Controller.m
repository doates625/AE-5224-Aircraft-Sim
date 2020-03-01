classdef (Abstract) Controller < AE5224.control.Controller
    %CONTROLLER Superclass for quadrotor controllers
    
    properties (Access = protected)
        p_pids; % Position PIDs [controls.PID]
        q_pids; % Attitude PIDs [controls.PID]
    end

    methods (Access = public)
        function obj = Controller(model, del_t)
            %obj = CONTROLLER(model, del_t)
            %   - model = Quadrotor model [AE5224.quad_rotor.Model]
            %   - del_t = Time delta [s]
            
            % Imports
            import('controls.PID');
            
            % Superconstructor
            obj@AE5224.control.Controller(model);
            
            % Position control
            f_ctrl = 1 / del_t;
            p_s = -3.0;
            p_kp = p_s^2;
            p_ki = 0;
            p_kd = -2*p_s;
            acc_max = 10.0;
            obj.p_pids = PID.empty;
            for i = 1:3
                obj.p_pids(i) = PID(...
                    p_kp, p_ki, p_kd, -acc_max, +acc_max, f_ctrl);
            end
            
            % Attitude control
            q_s = -6.0;
            q_kp = 2*q_s^2;
            q_ki = 0;
            q_kd = -3*q_s;
            alp_max = 20.0;
            obj.q_pids = PID.empty;
            for i = 1:3
                obj.q_pids(i) = PID(...
                    q_kp, q_ki, q_kd, -alp_max, +alp_max, f_ctrl);
            end
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x, p_e_ref, a_e_ref, th_z_ref)
            %u = UPDATE_(obj, x, p_e_ref, a_e_ref, th_z_ref)
            %   Compute controls from state and reference
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - p_e_ref = Position Earth ref [m]
            %   - a_e_ref = Accel Earth ref [m/s^2]
            %   - th_z_ref = Heading ref [rad]
            %   - u = Unsurated control vector
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            import('AE5224.const.get_g');
            import('quat.Quat');
            
            % Unpack state
            [p_e, q_e, ~, w_b] = unpack_x(x);
            q_e = Quat(q_e);
            
            % Unit vectors
            x_hat = [1; 0; 0];
            y_hat = [0; 1; 0];
            z_hat = [0; 0; 1];
            
            % Attitude setpoint
            a_e_fb = zeros(3, 1);
            for i = 1:3
                a_e_fb(i) = obj.p_pids(i).update(p_e_ref(i) - p_e(i));
            end
            a_e_fb = a_e_ref + a_e_fb - z_hat*get_g();
            sz = sin(th_z_ref);
            cz = cos(th_z_ref);
            aeh = a_e_fb / norm(a_e_fb);
            th_x_ref = -asin((sz * aeh(1) - cz * aeh(2)));
            th_y_ref = -asin((cz * aeh(1) + sz * aeh(2)) / cos(th_x_ref));
            q_z = Quat(z_hat, th_z_ref);
            q_y = Quat(y_hat, th_y_ref);
            q_x = Quat(x_hat, th_x_ref);
            q_e_ref = q_z * q_y * q_x;
            F_bz = dot(q_e.rotate(z_hat), obj.model.m * a_e_fb);
            
            % Attitude control
            q_err = q_e \ q_e_ref;
            q_err = q_err.pos_w();
            q_err = [q_err.x; q_err.y; q_err.z];
            alp_b = zeros(3, 1);
            for i = 1:3
                alp_b(i) = obj.q_pids(i).update(q_err(i));
            end
            I_b = obj.model.I_b;
            M_b = I_b * alp_b + cross(w_b, I_b * w_b);
            
            % Propeller speeds
            w_p2 = obj.model.mat \ [F_bz; M_b];
            w_p2 = max(w_p2, 0);
            u = sqrt(w_p2);
        end
    end
end