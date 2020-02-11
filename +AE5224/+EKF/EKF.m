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
        function obj = EKF()
            %obj = EKF()
        end
    end
end