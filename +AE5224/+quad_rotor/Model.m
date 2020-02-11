classdef Model < AE5224.rigid_body.Model
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
            obj@AE5224.rigid_body.Model(m, I_b);
            
            % Quadrotor constants
            obj.L = 0.175;
            obj.k_F = 1.50e-09;
            obj.k_M = 6.11e-08;
        end
        
        function [F_b, M_b] = forces(obj, x, u)
            %[F_b, M_b] = FORCES(obj, x, u)
            %   Compute body-frame net forces and moments
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   - u = Input vector [w_1; w_2; w_3; w_4]
            %   
            %   Outputs:
            %   - F_b = Net force vector [N]
            %   - M_b = Net moment vector [N*m]
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            import('AE5224.const.get_g');
            import('quat.Quat');
            
            % Unpack state vector
            [~, q_e, ~, ~] = unpack_x(x);
            
            % Convert input to FMs
            w_p2 = u.^2;
            F_wp = obj.k_F * w_p2;
            M_wp = obj.k_M * w_p2;
            
            % Net force body-frame
            F_mg_e = [0; 0; obj.m * get_g()];
            F_mg_b = Quat(q_e).inv().rotate(F_mg_e);
            F_wp_b = [0; 0; -sum(F_wp)];
            F_b = F_mg_b + F_wp_b;
            
            % Net moment body-frame
            M_b = zeros(3, 1);
            M_b(1) = obj.L * (F_wp(2) - F_wp(4));
            M_b(2) = obj.L * (F_wp(3) - F_wp(1));
            M_b(3) = [+1, -1, +1, -1] * M_wp;
        end
    end
end