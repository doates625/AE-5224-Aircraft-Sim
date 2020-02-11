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
    
    methods (Access = public)
        function obj = EKF(xh, Ex, Ew, Ea, Ep, Ev)
            %obj = EKF(xh, Ex, Eu, Ez)
            %   Construct UAV EKF
            %   - xh = Init state estimate [16 x 1]
            %   - Ex = Init state cov [16 x 16]
            %   - Ew = Angular velocity cov [3 x 3]
            %   - Ea = Acceleration cov [3 x 3]
            %   - Ep = Position cov [3 x 3]
            %   - Ev = Velocity cov [3 x 3]
            
            % Imports
            import('AE5224.EKF.f');
            import('AE5224.EKF.h');
            import('AE5224.EKF.Fx');
            import('AE5224.EKF.Fu');
            import('AE5224.EKF.Hx');
            
            % Input-output covariances
            Eu = zeros(6);
            Eu(1:3, 1:3) = Ew;
            Eu(4:6, 4:6) = Ea;
            Ez = zeros(6);
            Ez(1:3, 1:3) = Ep;
            Ez(4:6, 4:6) = Ev;
            
            % Constructor
            obj@kalman.EKF(xh, Ex, Eu, Ez, @f, @h, @Fx, @Fu, @Hx);
        end
    end
end