classdef Body < AE5224.rigid_body.Body
    %BODY Class for quadrotor aircraft rigid body models
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
        function obj = Body()
            %obj = BODY()
            %   Construct quadrotor aircraft model
            %   Uses parameters from Hummingbird UAV

            % Inertial constants
            m = 0.5;
            I_xx = 2.32e-03;
            I_yy = 2.32e-03;
            I_zz = 4.00e-03;
            I_b = diag([I_xx, I_yy, I_zz]);
            obj@AE5224.rigid_body.Body(m, I_b);
            
            % Quadrotor constants
            obj.L = 0.175;
            obj.k_F = 1.50e-09;
            obj.k_M = 6.11e-08;
        end
    end
end