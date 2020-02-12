classdef Log < handle
    %LOG State estimate log for UAV EKF
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        sim;    % UAV simulator [AE5224.rigid_body.Sim]
        ekf;    % Kalman filter [AE5224.EKF.EKF]
        t;      % Timestemp log [s]
        q_e;    % Attitude log [Earth, quat]
        p_e;    % Position log [Earth, m]
        v_e;    % Velocity log [Earth, m/s]
        w_e;    % Air velocity log [Earth, m/s]
        b_e;    % Magnetic field log [Earth, uT]
    end
    
    methods (Access = public)
        function obj = Log(sim, ekf)
            %obj = LOG(ekf)
            %   Construct log
            %   - sim = UAV simulator [AE5224.rigid_body.Sim]
            %   - ekf = Kalman filter [AE5224.EKF.EKF]
            obj.sim = sim;
            obj.ekf = ekf;
        end
        
        function update(obj)
            %UPDATE(obj) Add current state estimate to log
            
            % Imports
            import('AE5224.EKF.EKF.unpack_x');
            
            % Function
            [q_e_, p_e_, v_e_, w_e_, b_e_] = unpack_x(obj.ekf.x_est);
            obj.t = [obj.t, obj.sim.t];
            obj.q_e = [obj.q_e, q_e_];
            obj.p_e = [obj.p_e, p_e_];
            obj.v_e = [obj.v_e, v_e_];
            obj.w_e = [obj.w_e, w_e_];
            obj.b_e = [obj.b_e, b_e_];
        end
        
        function plot_path(obj)
            %PLOT_PATH(obj) Plots path in 3D space
            
            % Imports
            import('quat.Quat');
            import('live_plot.Frame3D');
            
            % Format figure
            hold on, grid on
            title('Body Path')
            xlabel('Pos-X [m]')
            ylabel('Pos-Y [m]')
            zlabel('Pos-Z [m]')
            
            % Plot position trajectory
            p_x = obj.p_e(1, :);
            p_y = obj.p_e(2, :);
            p_z = obj.p_e(3, :);
            plot3(p_x, p_y, p_z, 'k--')
            
            % Plot pose trajectory
            n = length(obj.t);
            del_n = ceil(1 / obj.sim.del_t);
            for i = 1 : del_n : n
                vec_len = max(1, 0.02 * max(range(obj.p_e, 2)));
                frame = Frame3D(vec_len);
                frame.plot_x.plot_.Color = 'r';
                frame.plot_y.plot_.Color = 'g';
                frame.plot_z.plot_.Color = 'b';
                frame.plot_x.plot_.LineStyle = '--';
                frame.plot_y.plot_.LineStyle = '--';
                frame.plot_z.plot_.LineStyle = '--';
                frame.plot_x.plot_.LineWidth = 2;
                frame.plot_y.plot_.LineWidth = 2;
                frame.plot_z.plot_.LineWidth = 2;
                R = Quat(obj.q_e(:, i)).mat_rot();
                p = obj.p_e(:, i);
                frame.update(R, p);
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