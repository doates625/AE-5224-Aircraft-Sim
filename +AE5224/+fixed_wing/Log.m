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
        function obj = Log(sim_body, sim_wind, ekf, n)
            %obj = LOG(sim_body, sim_wind, ekf, n)
            %   Construct new log
            %   - sim_body = Body simulator [AE5224.rigid_body.Sim]
            %   - sim_wind = Wind simulator [AE5224.Wind]
            %   - ekf = Kalman filter [AE5224.EKF]
            %   - n = Log pre-allocation length [cnts]
            obj@AE5224.Log(sim_body, sim_wind, ekf, n)
            obj.d_e = nan(1, n);
            obj.d_a = nan(1, n);
            obj.d_r = nan(1, n);
            obj.d_p = nan(1, n);
        end
        
        function update(obj, u, varargin)
            %UPDATE(obj, u, z_gyr, z_mag, z_gps)
            %   Add states and measurements to log
            %   - u = Control input [d_e; d_a; d_r; d_p]
            %   - z_gyr = Gyro reading [opt]
            %   - z_mag = Mag reading [opt]
            %   - z_air = Airspeed reading [opt]
            %   - z_gps = GPS reading [opt]
            obj.d_e(:, obj.log_i) = u(1);
            obj.d_a(:, obj.log_i) = u(2);
            obj.d_r(:, obj.log_i) = u(3);
            obj.d_p(:, obj.log_i) = u(4);
            update@AE5224.Log(obj, varargin{:});
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