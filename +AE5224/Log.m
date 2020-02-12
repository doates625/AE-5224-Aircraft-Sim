classdef Log < handle
    %LOG State log for rigid body simulations
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        % Data sources
        sim;    % Simulator [AE5224.rigid_body.Sim]
        ekf;    % Kalman filter [AE5224.EKF]
        log_t;  % Timestamp log [s]
        log_i;  % Log index [cnts]
        
        % Actual state logs
        p_e_act;    % Position Earth [m]
        q_e_act;    % Attitude Earth [quat]
        v_e_act;    % Velocity Earth [m/s]
        w_b_act;    % Angular velocity Body [rad/s]
        %   w_e_act;    % Air velocity Earth [m/s]
        b_e_act;    % Magnetic field Earth [uT]
        
        % Measurement logs
        p_e_mea;    % Position Earth [m]
        v_e_mea;    % Velocity Earth [m]
        %   w_b_mea;    % Angular velocity Body [rad/s]
        
        % Estimated state logs
        p_e_est;    % Position Earth [m]
        q_e_est;    % Attitude Earth [quat]
        v_e_est;    % Velocity Earth [m/s]
        w_e_est;    % Air velocity Earth [m/s]
        b_e_est;    % Magnetic field Earth [uT]
        
        % Estimate std logs
        p_e_std;    % Position Earth [m]
        q_e_std;    % Attitude Earth [quat]
        v_e_std;    % Velocity Earth [m/s]
        w_e_std;    % Air velocity Earth [m/s]
        b_e_std;    % Magnetic field Earth [uT]
    end
    
    methods (Access = public)
        function obj = Log(varargin)
            %LOG Construct simulation log
            %   
            %   obj = LOG(sim, ekf, t_max) Make new log
            %   - sim = Simulator [AE5224.rigid_body.Sim]
            %   - ekf = Kalman filter [AE5224.EKF]
            %   - t_max = Simulation duration [s]
            %   
            %   obj = LOG(file) Load log from file
            %   - file = Mat file name ['*.mat']
            if nargin == 3
                % Parse arguments
                obj.sim = varargin{1};
                obj.ekf = varargin{2};
                t_max = varargin{3};
                
                % Pre-allocate logs
                n = ceil(t_max / obj.sim.del_t) + 1;
                obj.log_t = nan(1, n);
                obj.p_e_act = nan(3, n);
                obj.q_e_act = nan(4, n);
                obj.v_e_act = nan(3, n);
                obj.w_b_act = nan(3, n);
                obj.b_e_act = nan(3, n);
                obj.p_e_mea = nan(3, n);
                obj.v_e_mea = nan(3, n);
                obj.p_e_est = nan(3, n);
                obj.q_e_est = nan(4, n);
                obj.v_e_est = nan(3, n);
                obj.w_e_est = nan(3, n);
                obj.b_e_est = nan(3, n);
                obj.p_e_std = nan(3, n);
                obj.q_e_std = nan(4, n);
                obj.v_e_std = nan(3, n);
                obj.w_e_std = nan(3, n);
                obj.b_e_std = nan(3, n);
                obj.log_i = 1;
                
                % Initial update
                obj.update();
            elseif nargin == 1
                % Load from file
                file = load(varargin{1});
                obj = file.obj;
            else
                error('Invalid nargin.');
            end
        end
        
        function update(obj, z_gps)
            %UPDATE(obj, z_gps)
            %   Add states and measurements to log
            %   - z_gps = GPS reading [opt]

            % Imports
            import('AE5224.const.get_b');
            import('AE5224.rigid_body.Model');
            import('AE5224.sensors.GPS');
            import('AE5224.EKF');
            
            % Simulation logs
            [p_e_act_, q_e_act_, v_e_act_, w_b_act_] = ...
                Model.unpack_x(obj.sim.x);
            obj.log_t(obj.log_i) = obj.sim.t;
            obj.p_e_act(:, obj.log_i) = p_e_act_;
            obj.q_e_act(:, obj.log_i) = q_e_act_;
            obj.v_e_act(:, obj.log_i) = v_e_act_;
            obj.w_b_act(:, obj.log_i) = w_b_act_;
            obj.b_e_act(:, obj.log_i) = get_b();
            
            % Measurement logs
            if nargin > 1
                [p_e_mea_, v_e_mea_] = GPS.unpack_z(z_gps);
                obj.p_e_mea(:, obj.log_i) = p_e_mea_;
                obj.v_e_mea(:, obj.log_i) = v_e_mea_;
            end
            
            % Kalman filter logs
            [q_e_est_, p_e_est_, v_e_est_, w_e_est_, b_e_est_] = ...
                EKF.unpack_x(obj.ekf.x_est);
            obj.p_e_est(:, obj.log_i) = p_e_est_;
            obj.q_e_est(:, obj.log_i) = q_e_est_;
            obj.v_e_est(:, obj.log_i) = v_e_est_;
            obj.w_e_est(:, obj.log_i) = w_e_est_;
            obj.b_e_est(:, obj.log_i) = b_e_est_;
            
            % Kalman filter stds
            x_std = sqrt(diag(obj.ekf.cov_x));
            [q_e_std_, p_e_std_, v_e_std_, w_e_std_, b_e_std_] = ...
                EKF.unpack_x(x_std);
            obj.p_e_std(:, obj.log_i) = p_e_std_;
            obj.q_e_std(:, obj.log_i) = q_e_std_;
            obj.v_e_std(:, obj.log_i) = v_e_std_;
            obj.w_e_std(:, obj.log_i) = w_e_std_;
            obj.b_e_std(:, obj.log_i) = b_e_std_;
            
            % Increment log counter
            obj.log_i = obj.log_i + 1;
        end
        
        function save(obj)
            %SAVE(obj) Trim and save object to 'log.mat'
            obj.log_i = obj.log_i - 2;
            range_i = 1:obj.log_i;
            obj.log_t = obj.log_t(range_i);
            obj.p_e_act = obj.p_e_act(:, range_i);
            obj.q_e_act = obj.q_e_act(:, range_i);
            obj.v_e_act = obj.v_e_act(:, range_i);
            obj.w_b_act = obj.w_b_act(:, range_i);
            obj.b_e_act = obj.b_e_act(:, range_i);
            obj.p_e_mea = obj.p_e_mea(:, range_i);
            obj.v_e_mea = obj.v_e_mea(:, range_i);
            obj.q_e_est = obj.q_e_est(:, range_i);
            obj.p_e_est = obj.p_e_est(:, range_i);
            obj.v_e_est = obj.v_e_est(:, range_i);
            obj.w_e_est = obj.w_e_est(:, range_i);
            obj.b_e_est = obj.b_e_est(:, range_i);
            obj.p_e_std = obj.p_e_std(:, range_i);
            obj.q_e_std = obj.q_e_std(:, range_i);
            obj.v_e_std = obj.v_e_std(:, range_i);
            obj.w_e_std = obj.w_e_std(:, range_i);
            obj.b_e_std = obj.b_e_std(:, range_i);
            save('log.mat', 'obj');
        end
        
        function plot(obj)
            %PLOT(obj) Generates all state plots
            obj.plot_p_e();
            obj.plot_v_e();
            obj.plot_q_e();
            obj.plot_traj();
        end
        
        function plot_p_e(obj)
            %PLOT_P_E(obj) Plot position Earth
            figure;
            lbs = 'XYZ';
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(['Position-', lbs(i)])
                xlabel('Time [s]')
                ylabel('Pos [m]')
                plot(obj.log_t, obj.p_e_act(i, :), 'k--')
                plot(obj.log_t, obj.p_e_est(i, :), 'b-')
                plot(obj.log_t, obj.p_e_est(i, :) + 2*obj.p_e_std(i, :), 'b--');
                plot(obj.log_t, obj.p_e_est(i, :) - 2*obj.p_e_std(i, :), 'b--');
                plot(obj.log_t, obj.p_e_mea(i, :), 'rx')
                legend('Act', 'Est', 'Est+', 'Est-', 'Mea')
                if i == 3
                    set(gca, 'YDir','reverse')
                end
            end
        end
        
        function plot_v_e(obj)
            %PLOT_V_E(obj) Plot velocity Earth
            figure;
            lbs = 'XYZ';
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(['Velocity-', lbs(i)])
                xlabel('Time [s]')
                ylabel('Vel [m/s]')
                plot(obj.log_t, obj.v_e_act(i, :), 'k--');
                plot(obj.log_t, obj.v_e_est(i, :), 'b-');
                plot(obj.log_t, obj.v_e_est(i, :) + 2*obj.v_e_std(i, :), 'b--');
                plot(obj.log_t, obj.v_e_est(i, :) - 2*obj.v_e_std(i, :), 'b--');
                plot(obj.log_t, obj.v_e_mea(i, :), 'rx');
                legend('Act', 'Est', 'Est+', 'Est-', 'Mea')
                if i == 3
                    set(gca, 'YDir','reverse')
                end
            end
        end
        
        function plot_q_e(obj)
            %PLOT_Q_E(obj) Plot attitude Earth
            figure;
            lbs = 'WXYZ';
            for i = 1:4
                subplot(4, 1, i)
                hold on, grid on
                title(['Attitude-', lbs(i)])
                xlabel('Time [s]')
                ylabel('Quat')
                plot(obj.log_t, obj.q_e_act(i, :), 'k--')
                plot(obj.log_t, obj.q_e_est(i, :), 'b-')
                plot(obj.log_t, obj.q_e_est(i, :) + 2*obj.q_e_std(i, :), 'b--');
                plot(obj.log_t, obj.q_e_est(i, :) - 2*obj.q_e_std(i, :), 'b--');
                legend('Act', 'Est', 'Est+', 'Est-')
            end
        end
        
        function plot_traj(obj)
            %PLOT_TRAJ(obj) Plots trajectory with attitude in 3D
            
            % Imports
            import('live_plot.Frame3D');
            import('quat.Quat');
            
            % Format figure
            figure;
            hold on, grid on
            title('Body Trajectory')
            xlabel('Pos-X [m]')
            ylabel('Pos-Y [m]')
            zlabel('Pos-Z [m]')
            
            % Plot position
            p_x_act = obj.p_e_act(1, :);
            p_y_act = obj.p_e_act(2, :);
            p_z_act = obj.p_e_act(3, :);
            p_x_est = obj.p_e_est(1, :);
            p_y_est = obj.p_e_est(2, :);
            p_z_est = obj.p_e_est(3, :);
            plot3(p_x_act, p_y_act, p_z_act, 'k-');
            plot3(p_x_est, p_y_est, p_z_est, 'k--');
            
            % Plot attitude
            n = length(obj.log_t);
            del_n = ceil(1 / obj.sim.del_t);
            vec_len = max(1, 0.02 * max(range(obj.p_e_act, 2)));
            for i = 1 : del_n : n
                % Attitude actual
                att_act = Frame3D(vec_len);
                att_act.plot_x.plot_.Color = 'r';
                att_act.plot_y.plot_.Color = 'g';
                att_act.plot_z.plot_.Color = 'b';
                att_act.plot_x.plot_.LineWidth = 2;
                att_act.plot_y.plot_.LineWidth = 2;
                att_act.plot_z.plot_.LineWidth = 2;
                R = Quat(obj.q_e_act(:, i)).mat_rot();
                p = obj.p_e_act(:, i);
                att_act.update(R, p);
                
                % Attitude estimate
                att_est = Frame3D(vec_len);
                att_est.plot_x.plot_.Color = 'r';
                att_est.plot_y.plot_.Color = 'g';
                att_est.plot_z.plot_.Color = 'b';
                att_est.plot_x.plot_.LineWidth = 2;
                att_est.plot_y.plot_.LineWidth = 2;
                att_est.plot_z.plot_.LineWidth = 2;
                att_est.plot_x.plot_.LineStyle = '--';
                att_est.plot_y.plot_.LineStyle = '--';
                att_est.plot_z.plot_.LineStyle = '--';
                R = Quat(obj.q_e_est(:, i)).mat_rot();
                p = obj.p_e_est(:, i);
                att_est.update(R, p);
            end
            
            % Format axes
            set(gca, 'YDir','reverse')
            set(gca, 'ZDir','reverse')
            view(-105, +25)
            axis equal
            camproj perspective
        end
    end
end