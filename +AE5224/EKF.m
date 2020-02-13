classdef EKF < kalman.EKF
    %EKF Extended Kalman Filter for UAV navigation
    %   
    %   State x:
    %   - q_e = Attitude Earth [quat]
    %   - p_e = Position Earth [m]
    %   - v_e = Velocity Earth [m/s]
    %   - w_e = Air velocity Earth [m/s]
    %   - b_e = Magnetic field Earth [uT]
    %   
    %   Input u:
    %   - w_b = Angular velocity Body [rad/s]
    %   - a_b = Acceleration Body [m/s^2]
    %   
    %   Output z:
    %   - p_e = Position Earth [m]
    %   - v_e = Velocity Earth [m]
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        del_t;  % Simulation time delta [s]
    end
    
    methods (Access = public, Static)
        function x = pack_x(q_e, p_e, v_e, w_e, b_e)
            %x = PACK_X(q_e, p_e, v_e, w_e, b_e)
            %   Make state vector from components
            x = [q_e; p_e; v_e; w_e; b_e];
        end
        
        function [q_e, p_e, v_e, w_e, b_e] = unpack_x(x)
            %[q_e, p_e, v_e, w_e, b_e] = UNPACK_X(x)
            %   Get components from state vector
            q_e = x(01:04);
            p_e = x(05:07);
            v_e = x(08:10);
            w_e = x(11:13);
            b_e = x(14:16);
        end
        
        function u = pack_u(w_b, a_b)
            %u = PACK_U(w_b, a_b)
            %   Make input vector from components
            u = [w_b; a_b];
        end
        
        function [w_b, a_b] = unpack_u(u)
            %[w_b, a_b] = UNPACK_U(u)
            %   Get components from input vector
            w_b = u(1:3);
            a_b = u(4:6);
        end
        
        function z = pack_z_gps(p_e, v_e)
            %z = PACK_Z_GPS(p_e, v_e)
            %   Make GPS output vector from components
            z = [p_e; v_e];
        end
        
        function [p_e, v_e] = unpack_z_gps(z)
            %[p_e, v_e] = UNPACK_Z_GPS(z)
            %   Get components from GPS output vector
            p_e = z(1:3);
            v_e = z(4:6);
        end
    end
    
    methods (Access = public)
        function obj = EKF(x_est, cov_x, ...
                cov_w_b, cov_a_b, cov_b_b, cov_v_a, cov_p_e, cov_v_e, del_t)
            %obj = EKF(x_est, cov_x, ...
            %       cov_w_b, cov_a_b, cov_b_b, cov_v_a, cov_p_e, cov_v_e, del_t)
            %   Construct UAV EKF
            %   - x_est = Init state estimate [16 x 1]
            %   - cov_x = Init state cov [16 x 16]
            %   - cov_w_b = Angular velocity cov [3 x 3]
            %   - cov_a_b = Acceleration cov [3 x 3]
            %   - cov_b_b = Magnetic field cov [3 x 3]
            %   - cov_v_a = Airspeed cov [3 x 3]
            %   - cov_p_e = GPS position cov [3 x 3]
            %   - cov_v_e = GPS velocity cov [3 x 3]
            %   - del_t = Filter time delta [s]
            
            % Input-output covariances
            cov_u = zeros(6);
            cov_u(1:3, 1:3) = cov_w_b;
            cov_u(4:6, 4:6) = cov_a_b;
            cov_z_mag = cov_b_b;
            cov_z_air = cov_v_a;
            cov_z_gps = zeros(6);
            cov_z_gps(1:3, 1:3) = cov_p_e;
            cov_z_gps(4:6, 4:6) = cov_v_e;
            cov_z = {cov_z_mag; cov_z_air; cov_z_gps};
            
            % EKF constructor
            f = @(x, u) AE5224.EKF.f_(x, u, del_t);
            h_mag = @(x) AE5224.EKF.h_mag_(x);
            h_air = @(x) AE5224.EKF.h_air_(x);
            h_gps = @(x) AE5224.EKF.h_gps_(x);
            h = {h_mag; h_air; h_gps};
            fx = @(x, u) AE5224.EKF.fx_(x, u, del_t);
            fu = @(x, u) AE5224.EKF.fu_(x, u, del_t);
            hx_mag = @(x) AE5224.EKF.hx_mag_(x);
            hx_air = @(x) AE5224.EKF.hx_air_(x);
            hx_gps = @(x) AE5224.EKF.hx_gps_(x);
            hx = {hx_mag; hx_air; hx_gps};
            obj@kalman.EKF(x_est, cov_x, cov_u, cov_z, f, h, fx, fu, hx);
        end
        
        function x_est = correct_mag(obj, z)
            %x_est = CORRECT_MAG(obj, z)
            %   Magnetometer correction step
            %   - z = Magnetometer outputs [b_bx; b_by; b_bz]
            %   - x_est = Corrected state [16 x 1]
            x_est = obj.correct(z, 1);
        end
        
        function x_est = correct_air(obj, z)
            %x_est = CORRECT_AIR(obj, z)
            %   Airspeed correction step
            %   - z = Airspeed outputs [v_ax; v_ay; v_az]
            %   - x_est = Corrected state [16 x 1]
            x_est = obj.correct(z, 2);
        end
        
        function x_est = correct_gps(obj, z)
            %x_est = CORRECT_GPS(obj, z)
            %   GPS correction step
            %   - z = GPS outputs [p_e; v_e]
            %   - x_est = Corrected state [16 x 1]
            x_est = obj.correct(z, 3);
        end
        
        function x_est = correct(obj, z, i)
            %x_est = CORRECT(obj, z)
            %   Correction step
            %   - z = Output vector [6 x 1]
            %   - i = Output index [1...3]
            %   - x_est = Corrected state [16 x 1]
            x_est = correct@kalman.EKF(obj, z, i);
            q_e = x_est(1:4);
            N = ones(16, 1);
            N(1:4) = 1 / norm(q_e);
            N = diag(N);
            obj.x_est = N * x_est;
            obj.cov_x = N * obj.cov_x * N;
            x_est = obj.x_est;
        end
    end

    methods (Access = protected, Static)
        function xn = f_(x, u, del_t)
            %xn = F_(x, u, del_t)
            %   State transition function
            %   - x = State vector
            %   - u = Input vector
            %   - del_t = Time delta [s]
            %   - xn = Next state

            % Imports
            import('AE5224.EKF.unpack_x');
            import('AE5224.EKF.unpack_u');
            import('AE5224.EKF.pack_x');
            import('AE5224.const.get_g');
            import('quat.Quat');

            % Unpack x and u
            [q_e, p_e, v_e, w_e, b_e] = unpack_x(x);
            [w_b, a_b] = unpack_u(u);

            % Attitude update
            del_q = Quat(w_b, norm(w_b) * del_t);
            q_e = Quat(q_e) * del_q;

            % Position update
            p_e = p_e + v_e * del_t;

            % Velocity update
            a_g = [0; 0; get_g()];
            a_e = q_e.rotate(a_b) + a_g;
            v_e = v_e + a_e * del_t;

            % Re-pack x
            q_e = q_e.vector();
            xn = pack_x(q_e, p_e, v_e, w_e, b_e);
        end
        
        function z = h_mag_(x)
            %z = H_MAG_(x)
            %   Magnetometer output function
            %   - x = State vector
            %   - z = Output vector [b_bx; b_by; b_bz]
            
            % Imports
            import('AE5224.EKF.unpack_x');
            import('quat.Quat');
            
            % Magnetometer output
            [q_e, ~, ~, ~, b_e] = unpack_x(x);
            z = Quat(q_e).inv().rotate(b_e);
        end
        
        function z = h_air_(x)
            %z = H_AIR_(x)
            %   Airspeed output function
            %   - x = State vector
            %   - z = Output vector [v_ax; v_ay; v_az]
            
            % Imports
            import('AE5224.EKF.unpack_x');
            import('quat.Quat');
            
            % Airspeed output
            [q_e, ~, v_e, w_e, ~] = unpack_x(x);
            z = Quat(q_e).inv().rotate(v_e - w_e);
        end
        
        function z = h_gps_(x)
            %z = H_GPS_(x)
            %   GPS output function
            %   - x = State vector
            %   - z = Output [p_e; v_e]
            
            % Imports
            import('AE5224.EKF.unpack_x');
            import('AE5224.EKF.pack_z_gps');
            
            % GPS output
            [~, p_e, v_e, ~, ~] = unpack_x(x);
            z = pack_z_gps(p_e, v_e);
        end
        
        function jac_xx = fx_(x, u, del_t)
            %jac_xx = FX_(x, u, del_t)
            %   Get state Jacobian
            %   - x = State vector
            %   - u = Input vector
            %   - del_t = Time delta [s]
            %   - jac_xx = State Jacobian

            % Imports
            import('AE5224.EKF.unpack_x');
            import('AE5224.EKF.unpack_u');
            import('quat.Quat');

            % Unpack x, u, del_t
            [q_e, ~, ~, ~, ~] = unpack_x(x);
            [w_b, a_b] = unpack_u(u);

            % Jacobian
            jac_xx = eye(16);
            jac_xx(1:4, 1:4) = Quat(w_b, norm(w_b) * del_t).mat_int();
            jac_xx(5:7, 8:10) = eye(3) * del_t;
            jac_xx(8:10, 1:4) = Quat(q_e).jac_rot(a_b) * del_t;
        end
        
        function jac_xu = fu_(x, u, del_t)
            %jac_xu = FU_(x, u, del_t)
            %   Get input Jacobian
            %   - x = State vector
            %   - u = Input vector
            %   - del_t = Time delta [s]
            %   - jac_xu = State Jacobian

            % Imports
            import('AE5224.EKF.unpack_x');
            import('AE5224.EKF.unpack_u');
            import('quat.Quat');

            % Unpack x and u
            [q_e, ~, ~, ~, ~] = unpack_x(x);
            [w_b, ~] = unpack_u(u);

            % Attitude from delta-q
            q_e = Quat(q_e);
            J_qe_dq = q_e.mat_ext();

            % Delta-q from theta-b
            norm_wb = norm(w_b);
            norm_wb_inv = 1 / norm_wb;
            del_th = 0.5 * norm_wb * del_t;
            sin_th = sin(del_th);
            cos_th = cos(del_th);
            wh = w_b * norm_wb_inv;
            J_dq_thb = zeros(4);
            J_dq_thb(1, 1) = -0.5 * sin_th;
            J_dq_thb(2:4, 1) = 0.5 * cos_th * wh;
            J_dq_thb(2:4, 2:4) = sin_th * eye(3);

            % Theta-b from w_b
            J_th_wb = norm_wb_inv * (w_b.' * del_t);
            J_wh_wb = norm_wb_inv * (eye(3) - wh * wh.');
            J_thb_wb = [J_th_wb; J_wh_wb];

            % Jacobian
            jac_xu = zeros(16, 6);
            jac_xu(1:4, 1:3) = J_qe_dq * J_dq_thb * J_thb_wb;
            jac_xu(8:10, 4:6) = q_e.mat_rot() * del_t;
        end
        
        function jac_zx = hx_mag_(x)
            %jac_zx = HX_MAG_(x)
            %   Get magnetometer output Jacobian
            %   - x = State vector
            %   - jac_zx = Output Jacobian
            
            % Imports
            import('AE5224.EKF.unpack_x');
            import('quat.Quat');
            
            % Unpack state vector
            [q_e, ~, ~, ~, b_e] = unpack_x(x);
            q_e_inv = Quat(q_e).inv();
            
            % Output Jacobain
            jac_zx = zeros(3, 16);
            jac_zx(:, 01:04) = q_e_inv.jac_rot(b_e);
            jac_zx(:, 14:16) = q_e_inv.mat_rot();
        end
        
        function jac_zx = hx_air_(x)
            %jac_zx = HX_AIR_(x)
            %   Get airspeed output Jacobian
            %   - x = State vector
            %   - jac_zx = Output Jacobian

            % Imports
            import('AE5224.EKF.unpack_x');
            import('quat.Quat');
            
            % Unpack state vector
            [q_e, ~, v_e, w_e, ~] = unpack_x(x);
            q_e_inv = Quat(q_e).inv();
            R_eb = q_e_inv.mat_rot();
            
            % Output Jacobain
            jac_zx = zeros(3, 16);
            jac_zx(:, 01:04) = q_e_inv.jac_rot(v_e - w_e);
            jac_zx(:, 08:10) = +R_eb;
            jac_zx(:, 11:13) = -R_eb;
        end
        
        function jac_zx = hx_gps_(~)
            %jac_zx = HX_GPS_(x)
            %   Get GPS output Jacobian
            %   - x = State vector
            %   - jac_zx = Output Jacobian
            jac_zx = [zeros(6, 4), eye(6, 6), zeros(6, 6)];
        end
    end
end