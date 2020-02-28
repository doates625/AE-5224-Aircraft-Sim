classdef Dubins < AE5224.control.Controller
    %DUBINS Dubins fixed-wing path tracking controller
    
    properties (SetAccess = protected)
        path;   % Dubins path [AE5224.dubins.path.Path]
        alt;    % Altitude [m]
        vel;    % Airspeed [m/s]
        ctrl;   % Linear controls [AE5224.fixed_wing.control.linear.Linear]
        x_st;   % Initial condition
    end
    
    methods (Access = public)
        function obj = Dubins(model, path, vel, alt)
            
            import('AE5224.fixed_wing.control.linear.Linear');
            import('AE5224.fixed_wing.trim.solve');
            import('AE5224.Trim');
            
            obj@AE5224.control.Controller(model);
            obj.path = path;
            obj.alt = alt;
            obj.vel = vel;
            obj.ctrl = Linear.empty(0, 1);
            
            s01 = obj.path.s01;
            s23 = obj.path.s23;
            rad = [s01; inf; s23] * obj.path.rad;
            for s = 1:3
                [x_st, u_st] = solve(model, Trim(vel, rad(s), 0, alt));
                obj.ctrl(s) = Linear(model, x_st, u_st);
                if s == 1
                    obj.x_st = x_st;
                end
            end
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x, t)
            %u = UPDATE(obj, x, t)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - t = Current time [s]
            %   - u = Unsurated control vector
            d = obj.vel * t;
            [~, th_z, s] = obj.path.get(d);
            u = obj.ctrl(s).update(obj.alt, th_z, x);
        end
    end
end