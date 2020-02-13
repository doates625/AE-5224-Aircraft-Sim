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
        w_e_act;    % Air velocity Earth [m/s]
        b_e_act;    % Magnetic field Earth [uT]
        
        % Measurement logs
        p_e_mea;    % Position Earth [m]
        v_e_mea;    % Velocity Earth [m]
        w_b_mea;    % Angular velocity Body [rad/s]*
        
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
                obj.w_e_act = nan(3, n);
                obj.b_e_act = nan(3, n);
                obj.p_e_mea = nan(3, n);
                obj.v_e_mea = nan(3, n);
                obj.w_b_mea = nan(3, n);
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
        
        function update(obj, z_gyr, z_gps)
            %UPDATE(obj, z_gyr, z_gps)
            %   Add states and measurements to log
            %   - z_gyr = Gyro reading [opt]
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
            obj.w_e_act(:, obj.log_i) = zeros(3, 1);
            obj.b_e_act(:, obj.log_i) = get_b();
            
            % Measurement logs
            if nargin >= 2
                obj.w_b_mea(:, obj.log_i) = z_gyr;
            end
            if nargin >= 3
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
            obj.w_e_act = obj.w_e_act(:, range_i);
            obj.b_e_act = obj.b_e_act(:, range_i);
            obj.p_e_mea = obj.p_e_mea(:, range_i);
            obj.v_e_mea = obj.v_e_mea(:, range_i);
            obj.w_b_mea = obj.w_b_mea(:, range_i);
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
            %PLOT(obj) Generate all state plots
            obj.plot_p_e();
            obj.plot_q_e();
            obj.plot_v_e();
            obj.plot_w_b();
            obj.plot_w_e();
            obj.plot_b_e();
            obj.plot_traj();
        end
        
        function plot_p_e(obj)
            %PLOT_P_E(obj) Plot position Earth
            obj.plot_state('Position Earth', 'Pos [m]', 'XYZ', {...
                obj.p_e_act, ...
                obj.p_e_est, ...
                obj.p_e_std, ...
                obj.p_e_mea});
        end
        
        function plot_q_e(obj)
            %PLOT_Q_E(obj) Plot attitude Earth
            obj.plot_state('Attitude Earth', 'Quat', 'WXYZ', {...
                obj.q_e_act, ...
                obj.q_e_est, ...
                obj.q_e_std});
        end
        
        function plot_v_e(obj)
            %PLOT_V_E(obj) Plot velocity Earth
            obj.plot_state('Velocity Earth', 'Vel [m/s]', 'XYZ', {...
                obj.v_e_act, ...
                obj.v_e_est, ...
                obj.v_e_std, ...
                obj.v_e_mea});
        end
        
        function plot_w_b(obj)
            %PLOT_W_B(obj) Plot angular velocity Body
            obj.plot_state('Angular Velocity Body', 'Vel [rad/s]', 'XYZ', {...
                obj.w_b_act, ...
                obj.w_b_mea});
        end
        
        function plot_w_e(obj)
            %PLOT_W_E(obj) Plot wind velocity Earth
            obj.plot_state('Wind Velocity Earth', 'Vel [m/s]', 'XYZ', {...
                obj.w_e_act, ...
                obj.w_e_est, ...
                obj.w_e_std});
        end
        
        function plot_b_e(obj)
            %PLOT_B_E(obj) Plot magnetic field Earth
            obj.plot_state('Magnetic Field Earth', 'Field [uT]', 'XYZ', {...
                obj.b_e_act, ...
                obj.b_e_est, ...
                obj.b_e_std});
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
    
    methods (Access = protected)
        function plot_state(obj, title_, ylabel_, axes_, x_logs)
            %PLOT_STATE(obj, title_, label_, axes_, x_logs);
            %   Plot log data
            %   
            %   Inputs:
            %   - title_ = Plot title
            %   - ylabel_ = Vertical axis label
            %   - axes_ = Axis letters ['char']
            %   - x_logs = Log cell array
            %   
            %   Log options:
            %   - {x_act, x_est, x_std, x_mea}
            %   - {x_act, x_est, x_std}
            %   - {x_act, x_mea}
            
            % Unpack logs
            n_logs = length(x_logs);
            x_act = x_logs{1};
            switch n_logs
                case 2
                    x_mea = x_logs{2};
                case 3
                    x_est = x_logs{2};
                    x_std = x_logs{3};
                case 4
                    x_est = x_logs{2};
                    x_std = x_logs{3};
                    x_mea = x_logs{4};
            end
            
            % Generate plots
            figure;
            n = length(axes_);
            for i = 1:n
                % Format subplot
                subplot(n, 1, i);
                hold on, grid on
                title([title_ ' ' axes_(i)])
                xlabel('Time [s]')
                ylabel(ylabel_)
                
                % Generate plots
                if n_logs == 2 || n_logs == 4
                    if n_logs == 2, ls = 'b-'; end
                    if n_logs == 4, ls = 'rx'; end
                    plot(obj.log_t, x_mea(i, :), ls, 'DisplayName', 'Mea');
                end
                if n_logs >= 3
                    plot(obj.log_t, x_est(i, :), 'b-', 'DisplayName', 'Est');
                    plot(obj.log_t, x_est(i, :) + 2*x_std(i, :), 'b--', ...
                        'DisplayName', 'Est+');
                    plot(obj.log_t, x_est(i, :) - 2*x_std(i, :), 'b--', ...
                        'DisplayName', 'Est-');
                end
                plot(obj.log_t, x_act(i, :), 'k--', 'DisplayName', 'Act');
                legend();
            end
            
            % Flip z-axis
            set(gca, 'YDir','reverse')
        end
    end
end