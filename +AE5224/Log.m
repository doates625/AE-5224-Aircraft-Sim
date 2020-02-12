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
        b_e_act;    % Magnetic field Earth [uT]
        
        % Measurement logs
        % p_e_mea;    % Position Earth [m]
        % v_e_mea;    % Velocity Earth [m]
        
        % Estimated state logs
        q_e_est;    % Attitude Earth [quat]
        p_e_est;    % Position Earth [m]
        v_e_est;    % Velocity Earth [m/s]
        w_e_est;    % Air velocity Earth [m/s]
        b_e_est;    % Magnetic field Earth [uT]
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
                n = floor(t_max / obj.sim.del_t);
                obj.log_t = nan(1, n);
                obj.p_e_act = nan(3, n);
                obj.q_e_act = nan(4, n);
                obj.v_e_act = nan(3, n);
                obj.w_b_act = nan(3, n);
                obj.b_e_act = nan(3, n);
                obj.q_e_est = nan(4, n);
                obj.p_e_est = nan(3, n);
                obj.v_e_est = nan(3, n);
                obj.w_e_est = nan(3, n);
                obj.b_e_est = nan(3, n);
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
        
        function update(obj)
            %UPDATE(obj, z_gps) Add states and measurements to log
            
            % Imports
            import('AE5224.const.get_b');
            import('AE5224.rigid_body.Model');
            import('AE5224.EKF');
            
            % Unpack state and estimate
            [p_e_act_, q_e_act_, v_e_act_, w_b_act_] = ...
                Model.unpack_x(obj.sim.x);
            [q_e_est_, p_e_est_, v_e_est_, w_e_est_, b_e_est_] = ...
                EKF.unpack_x(obj.ekf.x_est);
            
            % Simulation logs
            obj.log_t(obj.log_i) = obj.sim.t;
            obj.p_e_act(:, obj.log_i) = p_e_act_;
            obj.q_e_act(:, obj.log_i) = q_e_act_;
            obj.v_e_act(:, obj.log_i) = v_e_act_;
            obj.w_b_act(:, obj.log_i) = w_b_act_;
            obj.b_e_act(:, obj.log_i) = get_b();
            
            % Kalman filter logs
            obj.q_e_est(:, obj.log_i) = q_e_est_;
            obj.p_e_est(:, obj.log_i) = p_e_est_;
            obj.v_e_est(:, obj.log_i) = v_e_est_;
            obj.w_e_est(:, obj.log_i) = w_e_est_;
            obj.b_e_est(:, obj.log_i) = b_e_est_;
            
            % Increment log counter
            obj.log_i = obj.log_i + 1;
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
                legend('Act', 'Est')
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
                legend('Act', 'Est')
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
                legend('Act', 'Est')
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
        
        function save(obj)
            %SAVE(obj) Save object to 'log.mat'
            save('log.mat', 'obj');
        end
    end
end