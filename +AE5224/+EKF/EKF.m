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
        
        function z = pack_z(p_e, v_e)
            %z = PACK_Z(p_e, v_e)
            %   Make output vector from components
            z = [p_e; v_e];
        end
        
        function [p_e, v_e] = unpack_z(z)
            %[p_e, v_e] = UNPACK_Z(z)
            %   Get components from output vector
            p_e = z(1:3);
            v_e = z(4:6);
        end
    end
    
    methods (Access = public)
        function obj = EKF(x_est, cov_x, cov_w, cov_a, cov_p, cov_v, del_t)
            %obj = EKF(x_est, cov_x, cov_w, cov_a, cov_p, cov_v, del_t)
            %   Construct UAV EKF
            %   - ( = Init state estimate [16 x 1]
            %   - cov_x = Init state cov [16 x 16]
            %   - cov_w = Angular velocity cov [3 x 3]
            %   - cov_a = Acceleration cov [3 x 3]
            %   - cov_p = Position cov [3 x 3]
            %   - cov_v = Velocity cov [3 x 3]
            %   - del_t = Filter time delta [s]
            
            % Input-output covariances
            cov_u = zeros(6);
            cov_u(1:3, 1:3) = cov_w;
            cov_u(4:6, 4:6) = cov_a;
            cov_z = zeros(6);
            cov_z(1:3, 1:3) = cov_p;
            cov_z(4:6, 4:6) = cov_v;
            
            % EKF constructor
            obj@kalman.EKF(x_est, cov_x, cov_u, cov_z, [], [], [], [], []);
            obj.f = @(x, u) obj.f_(x, u);
            obj.h = @(x) obj.h_(x);
            obj.fx = @(x, u) obj.fx_(x, u);
            obj.fu = @(x, u) obj.fu_(x, u);
            obj.hx = @(x) obj.hx_(x);
            
            % Time delta
            obj.del_t = del_t;
        end
        
        function x_est = correct(obj, z)
            %x_est = CORRECT(obj, z)
            %   Correction step
            %   - z = Output vector [6 x 1]
            %   - x_est = Corrected state [16 x 1]
            x_est = correct@kalman.EKF(obj, z);
            q_e = x_est(1:4);
            N = ones(16, 1);
            N(1:4) = 1 / norm(q_e);
            N = diag(N);
            obj.x_est = N * x_est;
            obj.cov_x = N * obj.cov_x * N;
        end
    end
    
    methods (Access = protected)
        function xn = f_(obj, x, u)
            %xn = F_(obj, x, u)
            %   State transition function
            %   - x = State vector
            %   - u = Input vector
            %   - xn = Next state

            % Imports
            import('AE5224.const.get_g');
            import('quat.Quat');

            % Unpack x and u
            [q_e, p_e, v_e, w_e, b_e] = obj.unpack_x(x);
            [w_b, a_b] = obj.unpack_u(u);

            % Attitude update
            del_q = Quat(w_b, norm(w_b) * obj.del_t);
            q_e = Quat(q_e) * del_q;

            % Position update
            p_e = p_e + v_e * obj.del_t;

            % Velocity update
            a_g = [0; 0; get_g()];
            a_e = q_e.rotate(a_b) + a_g;
            v_e = v_e + a_e * obj.del_t;

            % Re-pack x
            q_e = q_e.vector();
            xn = obj.pack_x(q_e, p_e, v_e, w_e, b_e);
        end
        
        function z = h_(obj, x)
            %z = H_(obj, x)
            %   Output function
            %   - x = State vector
            %   - z = Output vector
            [~, p_e, v_e, ~, ~] = obj.unpack_x(x);
            z = obj.pack_z(p_e, v_e);
        end
        
        function jac_xx = fx_(obj, x, u)
            %jac_xx = FX_(obj, x, u)
            %   Get state Jacobian
            %   - x = State vector
            %   - u = Input vector
            %   - jac_xx = State Jacobian

            % Imports
            import('quat.Quat');

            % Unpack x, u, del_t
            [q_e, ~, ~, ~, ~] = obj.unpack_x(x);
            [w_b, a_b] = obj.unpack_u(u);

            % Jacobian
            jac_xx = eye(16);
            jac_xx(1:4, 1:4) = Quat(w_b, norm(w_b) * obj.del_t).mat_int();
            jac_xx(7:9, 8:10) = eye(3) * obj.del_t;
            jac_xx(8:10, 1:4) = Quat(q_e).jac_rot(a_b) * obj.del_t;
        end
        
        function jac_xu = fu_(obj, x, u)
            %jac_xu = FU_(obj, x, u)
            %   Get input Jacobian
            %   - x = State vector
            %   - u = Input vector
            %   - jac_xu = State Jacobian

            % Imports
            import('quat.Quat');

            % Unpack x and u
            [q_e, ~, ~, ~, ~] = obj.unpack_x(x);
            [w_b, ~] = obj.unpack_u(u);

            % Attitude from delta-q
            q_e = Quat(q_e);
            J_qe_dq = q_e.mat_ext();

            % Delta-q from theta-b
            norm_wb = norm(w_b);
            norm_wb_inv = 1 / norm_wb;
            del_th = 0.5 * norm_wb * obj.del_t;
            sin_th = sin(del_th);
            cos_th = cos(del_th);
            wh = w_b * norm_wb_inv;
            J_dq_thb = zeros(4);
            J_dq_thb(1, 1) = -0.5 * sin_th;
            J_dq_thb(2:4, 1) = 0.5 * cos_th * wh;
            J_dq_thb(2:4, 2:4) = sin_th * eye(3);

            % Theta-b from w_b
            J_th_wb = norm_wb_inv * (w_b.' * obj.del_t);
            J_wh_wb = norm_wb_inv * (eye(3) - wh * wh.');
            J_thb_wb = [J_th_wb; J_wh_wb];

            % Jacobian
            jac_xu = zeros(16, 6);
            jac_xu(1:4, 1:3) = J_qe_dq * J_dq_thb * J_thb_wb;
            jac_xu(8:10, 4:6) = q_e.mat_rot() * obj.del_t;
        end
        
        function jac_zx = hx_(~, ~)
            %jac_zx = HX_(obj, x)
            %   Get output Jacobian
            %   - x = State vector
            %   - jac_zx = Output Jacobian
            jac_zx = [zeros(6, 4), eye(6, 6), zeros(6, 6)];
        end
    end
end