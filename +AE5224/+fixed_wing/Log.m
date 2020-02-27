classdef Log < AE5224.Log
    %LOG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)
        d_e;    % Elevator angle [rad]
        d_a;    % Aileron angle [rad]
        d_r;    % Rudder angle [rad]
        d_p;    % Prop throttle [0-1]
    end
    
    methods (Access = public)
        function obj = Log(sim, ekf, n)
            %obj = LOG(sim, ekf, n)
            %   Construct new log
            %   - sim = Simulator [AE5224.rigid_body.Sim]
            %   - ekf = Kalman filter [AE5224.EKF]
            %   - n = Log pre-allocation length [cnts]
            obj@AE5224.Log(sim, ekf, n)
            obj.d_e = nan(1, n);
            obj.d_a = nan(1, n);
            obj.d_r = nan(1, n);
            obj.d_p = nan(1, n);
        end
        
        function update(obj, z_gyr, z_mag, z_gps, u)
            %UPDATE(obj, z_gyr, z_mag, z_gps, u)
            %   Add states and measurements to log
            %   - z_gyr = Gyro reading [opt]
            %   - z_mag = Mag reading [opt]
            %   - z_gps = GPS reading [opt]
            %   - u = Control input [d_e; d_a; d_r; d_p]
            obj.d_e(:, obj.log_i) = u(1);
            obj.d_a(:, obj.log_i) = u(2);
            obj.d_r(:, obj.log_i) = u(3);
            obj.d_p(:, obj.log_i) = u(4);
            update@AE5224.Log(obj, z_gyr, z_mag, z_gps);
        end
        
        function plot(obj)
            %PLOT(obj) Generate all state plots
            obj.plot_u();
            plot@AE5224.Log(obj);
        end
        
        function plot_u(obj)
            %PLOT_U(obj) Plots control inputs
            figure;
            
            % Elevator
            subplot(2, 2, 1)
            hold on, grid on
            title('Elevator')
            xlabel('Time [s]')
            ylabel('Angle [rad]')
            plot(obj.log_t, obj.d_e, 'r-')
            
            % Aileron
            subplot(2, 2, 2)
            hold on, grid on
            title('Aileron')
            xlabel('Time [s]')
            ylabel('Angle [rad]')
            plot(obj.log_t, obj.d_a, 'r-')
            
            % Rudder
            subplot(2, 2, 3)
            hold on, grid on
            title('Rudder')
            xlabel('Time [s]')
            ylabel('Angle [rad]')
            plot(obj.log_t, obj.d_r, 'r-')
            
            % Propeller
            subplot(2, 2, 4)
            hold on, grid on
            title('Propeller')
            xlabel('Time [s]')
            ylabel('Throttle [0-1]')
            plot(obj.log_t, obj.d_p, 'r-')
        end
    end
end