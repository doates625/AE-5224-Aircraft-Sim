classdef Log < handle
    %LOG State log for rigid body simulations
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        sim;    % Simulator [AE5224.rigid_body.Sim]
        t;      % Timestamp log [s]
        p_e;    % Position log [Earth, m]
        q_e;    % Attitude log [Earth, quaternion]
        v_e;    % Linear velocity [Earth, m]
        w_b;    % Angular velocity [Body, rad/s]
    end
    
    methods (Access = public)
        function obj = Log(arg)
            %LOG Construct simulation log
            %   
            %   obj = LOG(sim) Construct from AE5224.rigid_body.Sim
            %   obj = LOG(file) Construct from '*.mat' file
            if isa(arg, 'AE5224.rigid_body.Sim')
                obj.sim = arg;
                obj.update();
            else
                file = load(arg);
                obj = file.obj;
            end
        end
        
        function update(obj)
            %UPDATE(obj) Add current sim state to log
            obj.t = [obj.t, obj.sim.t];
            obj.p_e = [obj.p_e, obj.sim.p_e];
            obj.q_e = [obj.q_e, obj.sim.q_e];
            obj.v_e = [obj.v_e, obj.sim.v_e];
            obj.w_b = [obj.w_b, obj.sim.w_b];
        end
        
        function save(obj, file)
            %SAVE(obj, file) Saves log to given file ['*.mat']
            if nargin < 2, file = 'log.mat'; end
            save(file, 'obj');
        end
        
        function plot(obj)
            %PLOT(obj) Generates all state plots
            obj.plot_p();
            obj.plot_q();
            obj.plot_w();
            obj.plot_path();
        end
        
        function plot_p(obj)
            %PLOT_P(obj) Plots position and velocity
            figure;
            lbs = 'XYZ';
            for i = 1:3
                % Plot position
                subplot(2, 3, i)
                hold on, grid on
                title(['Position-', lbs(i)])
                xlabel('Time [s]')
                ylabel('Pos [m]')
                plot(obj.t, obj.p_e(i, :), 'b-')
                
                % Plot velocity
                subplot(2, 3, i+3)
                hold on, grid on
                title(['Velocity-', lbs(i)])
                xlabel('Time [s]')
                ylabel('Vel [m]')
                plot(obj.t, obj.v_e(i, :), 'r-')
            end
        end
        
        function plot_q(obj)
            %PLOT_Q(obj) Plots attitude quaternion
            figure;
            lbs = 'WXYZ';
            for i = 1:4
                subplot(4, 1, i)
                hold on, grid on
                title(['Attitude-', lbs(i)])
                xlabel('Time [s]')
                ylabel('Quat')
                plot(obj.t, obj.q_e(i, :), 'b-')
            end
        end
        
        function plot_w(obj)
            %PLOT_W(obj) Plots angular velocity
            figure;
            lbs = 'XYZ';
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(['Angular-', lbs(i)])
                xlabel('Time [s]')
                ylabel('Vel [rad/s]')
                plot(obj.t, obj.w_b(i, :), 'r-')
            end
        end
        
        function plot_path(obj)
            %PLOT_PATH(obj) Plots path in 3D space
            
            % Imports
            import('quat.Quat');
            import('live_plot.Frame3D');
            
            % Format figure
            figure;
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