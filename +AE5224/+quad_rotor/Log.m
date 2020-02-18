classdef Log < AE5224.Log
    %LOG State and control log for quadrotors
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        w_p;    % Prop speeds [rpm]
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
            obj.w_p = nan(4, n);
        end
        
        function update(obj, u, varargin)
            %UPDATE(obj, u, z_gyr, z_mag, z_gps)
            %   Add states and measurements to log
            %   - u = Control input [w_p1; w_p2; w_p3; w_p4]
            %   - z_gyr = Gyro reading [opt]
            %   - z_mag = Mag reading [opt]
            %   - z_air = Airspeed reading [opt]
            %   - z_gps = GPS reading [opt]
            obj.w_p(:, obj.log_i) = u;
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
            lbs = '1234';
            for i = 1:4
                subplot(2, 2, i);
                hold on, grid on
                title(['Prop Speed ' lbs(i)])
                xlabel('Time [s]')
                ylabel('Speed [rpm]')
                plot(obj.log_t, obj.w_p(i, :), 'r-')
            end
        end
    end
end