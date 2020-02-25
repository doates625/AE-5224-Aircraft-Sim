classdef TrajPlan < handle
    %TRAJPLAN Polynomial trajectory plan
    
    properties (Access = public)    % TODO make protected
        p_mat;  % Position matrix [3 x n+1]
        v_mat;  % Velocity matrix [3 x n+1]
        a_mat;  % Accel matrix [3 x n+1]
        j_mat;  % Jerk matrix [3 x n+1]
        s_mat;  % Snap matrix [3 x n+1]
        n;      % Polynomial order [int]
    end
    
    methods (Access = public)
        function obj = TrajPlan(p_mat)
            %obj = TRAJPLAN(p_mat, t_max)
            %   Create trajectory plan
            %   
            %   Position matrix p_mat satisfies:
            %   p(t) = p_mat * [t^0, t^1, t^2, ... t^n]
            obj.n = size(p_mat, 2) - 1;
            rng = 1:obj.n;
            obj.p_mat = p_mat;
            obj.v_mat = [rng .* obj.p_mat(:, 2:end), zeros(3, 1)];
            obj.a_mat = [rng .* obj.v_mat(:, 2:end), zeros(3, 1)];
            obj.j_mat = [rng .* obj.a_mat(:, 2:end), zeros(3, 1)];
            obj.s_mat = [rng .* obj.j_mat(:, 2:end), zeros(3, 1)];
        end
        
        function [p, v, a, j, s] = get(obj, t)
            %[p, v, a, j, s] = GET(obj, t)
            %   Get trajectory at time [t]
            %   
            %   Inputs:
            %   - t = Time vector [s, 1 x k]
            %   
            %   Outputs:
            %   - p = Position [m, 3 x k]
            %   - v = Velocity [m/s, 3 x k]
            %   - a = Accel [m/s^, 3 x k]
            %   - j = Jerk [m/s^3, 3 x k]
            %   - s = Snap [m/s^4, 3 x k]
            T = t.^((0:obj.n).');
            p = obj.p_mat * T;
            v = obj.v_mat * T;
            a = obj.a_mat * T;
            j = obj.j_mat * T;
            s = obj.s_mat * T;
        end
        
        function plot(obj, t_min, t_max, t_n, fig)
            %PLOT(obj, t_min, t_max, t_n, fig)
            %   Plot position as function of time
            %   - t_min = Min time [s]
            %   - t_max = Max time [s]
            %   - t_n = Sample count [int, def = 100]
            %   - fig = Figure handle [Figure, def = gcf]
            
            % Default args
            if nargin < 4, t_n = 100; end
            if nargin < 5, fig = gcf; end
            
            % Generate points
            t = linspace(t_min, t_max, t_n);
            [p, v] = obj.get(t);
            
            % Generate plot
            figure(fig); clf;
            lbs = 'XYZ';
            for i = 1:3
                subplot(3, 1, i)
                hold on, grid on
                title(['Trajectory ' lbs(i)])
                xlabel('Time [s]')
                plot(t, p(i, :), 'b-', 'DisplayName', 'Pos [m]');
                plot(t, v(i, :), 'r-', 'DisplayName', 'Vel [m/s]');
                legend;
            end
        end
    end
end