classdef Poly < AE5224.quad_rotor.control.Controller
    %POLY Quadrotor polynomial trajectory tracking controller
    
    properties (Access = protected)
        p_mat;  % Position matrix [3 x n+1]
        v_mat;  % Velocity matrix [3 x n+1]
        a_mat;  % Accel matrix [3 x n+1]
        n;      % Polynomial order [int]
    end
    
    methods (Access = public)
        function obj = Poly(model, p_mat, del_t)
            %obj = POLY(model, p_mat, 
            %   - model = Quadrotor model [AE5224.quad_rotor.Model]
            %   - del_t = Time delta [s]
            obj@AE5224.quad_rotor.control.Controller(model, del_t);
            obj.n = size(p_mat, 2) - 1;
            rng = 1:obj.n;
            obj.p_mat = p_mat;
            obj.v_mat = [rng .* obj.p_mat(:, 2:end), zeros(3, 1)];
            obj.a_mat = [rng .* obj.v_mat(:, 2:end), zeros(3, 1)];
        end
        
        function [p_e, v_e, a_e] = get(obj, t)
            %[p_e, v_e, a_e] = GET(obj, t)
            %   Get trajectory at given time
            %   - t = Time [s]
            %   - p_e = Position Earth [m]
            %   - v_e = Velocity Earth [m/s]
            %   - a_e = Accel Earth [m/s^2]
            T = t.^((0:obj.n).');
            p_e = obj.p_mat * T;
            v_e = obj.v_mat * T;
            a_e = obj.a_mat * T;
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x, t)
            %u = UPDATE_(obj, x, t)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - t = Current time [s]
            %   - u = Unsurated control vector

            % Reference signals
            [p_e_ref, ~, a_e_ref] = obj.get(t);
            th_z_ref = 0;

            % Control signals
            u = update_@AE5224.quad_rotor.control.Controller(...
                obj, x, p_e_ref, a_e_ref, th_z_ref);
        end
    end
end