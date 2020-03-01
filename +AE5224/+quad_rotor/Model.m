classdef Model < AE5224.Model
    %MODEL Quadrotor aircraft rigid body model
    %   
    %   Modeling assumptions:
    %   - Instant motor speed control
    %   - No aerodynamic drag
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        L;      % CM-to-prop distance [m]
        k_F;    % Motor force constant [N/rpm^2]
        k_M;    % Motor torque constant [N*m/rpm^2]
        mat;    % Force-moment matrix
    end
    
    methods (Access = public)
        function obj = Model()
            %obj = MODEL()
            %   Construct quadrotor aircraft model
            %   Uses parameters from Hummingbird UAV

            % Inertial constants
            m = 0.5;
            I_xx = 2.32e-03;
            I_yy = 2.32e-03;
            I_zz = 4.00e-03;
            I_b = diag([I_xx, I_yy, I_zz]);
            u_min = zeros(4, 1);
            u_max = 9000 * ones(4, 1);
            obj@AE5224.Model(m, I_b, u_min, u_max);
            
            % Quadrotor constants
            obj.L = 0.175;
            obj.k_F = 6.11e-08;
            obj.k_M = 1.50e-09;
            obj.mat = zeros(4);
            obj.mat(1, :) = -obj.k_F;
            obj.mat(2, 2) = +obj.L * obj.k_F;
            obj.mat(2, 4) = -obj.L * obj.k_F;
            obj.mat(3, 1) = -obj.L * obj.k_F;
            obj.mat(3, 3) = +obj.L * obj.k_F;
            obj.mat(4, 1) = +obj.k_M;
            obj.mat(4, 2) = -obj.k_M;
            obj.mat(4, 3) = +obj.k_M;
            obj.mat(4, 4) = -obj.k_M;
        end
        
        function [F_b, M_b] = forces(obj, x, u, ~)
            %[F_b, M_b] = FORCES(obj, x, u, w)
            %   Compute body-frame net forces and moments
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   - u = Input vector [w_p1; w_p2; w_p3; w_p4]
            %   - w = Disturbance [va_b]
            %   
            %   Outputs:
            %   - F_b = Net force vector [N]
            %   - M_b = Net moment vector [N*m]
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            import('AE5224.const.get_g');
            import('quat.Quat');
            
            % Unpack x and u
            [~, q_e, ~, ~] = unpack_x(x);
            w_p2 = u.^2;
            
            % Convert input to FMs
            FM = obj.mat * w_p2;
            F_wp_b = [0; 0; FM(1)];
            F_mg_e = [0; 0; obj.m * get_g()];
            F_mg_b = Quat(q_e).inv().rotate(F_mg_e);
            F_b = F_mg_b + F_wp_b;
            M_b = FM(2:4);
        end
    end
end