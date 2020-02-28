classdef Dubins < AE5224.control.Controller
    %DUBINS Dubins fixed-wing path tracking controller
    
    properties (SetAccess = protected)
        path;   % Dubins path [AE5224.dubins.path.Path]
        alt;    % Altitude [m]
        vel;    % Airspeed [m/s]
        h_max;  % Max heading dev [rad]
        h_gain; % Heading gain [rad/m]
    end
    
    properties (Access = protected)
        ctrl;   % Linear controls [AE5224.fixed_wing.control.linear.Linear]
        sect;   % Dubins path section [1...3]
        fin;    % Finished flag [logical]
    end
    
    methods (Access = public)
        function obj = Dubins(model, path, vel, alt)
            %obj = DUBINS(model, path, vel, alt)
            %   Create dubins tracking controller
            %   - model = Aircraft model [AE5224.fixed_wing.Model]
            %   - alt = Target altitude [m]
            %   - vel = Airspeed [m/s]
            
            % Imports
            import('AE5224.fixed_wing.control.linear.Linear');
            import('AE5224.fixed_wing.trim.solve');
            import('AE5224.Trim');
            
            % Construction
            obj@AE5224.control.Controller(model);
            obj.path = path;
            obj.alt = alt;
            obj.vel = vel;
            obj.h_max = pi/4;
            obj.h_gain = obj.h_max / 100;
            obj.sect = 1;
            obj.fin = false;
            
            % Linear controls
            obj.ctrl = Linear.empty(0, 1);
            sgn_a = obj.path.sgn_a();
            sgn_b = obj.path.sgn_b();
            rad = [sgn_a; inf; sgn_b] * obj.path.rad;
            for s = 1:3
                [x_st, u_st] = solve(model, Trim(vel, rad(s), 0, alt));
                obj.ctrl(s) = Linear(model, x_st, u_st);
            end
        end
        
        function fin = finished(obj)
            %fin = FINISHED(obj)
            %   Return true if path has finished
            fin = obj.fin;
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x, ~)
            %u = UPDATE(obj, x, ~)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - t = Current time [s]
            %   - u = Unsurated control vector
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            import('controls.clamp');
            
            % Get position
            [p_e, ~, ~, ~] = unpack_x(x);
            p_e = p_e(1:2);
            
            % Dubins state machine
            while ~obj.path.in_sect(p_e, obj.sect)
                obj.sect = obj.sect + 1;
                if obj.sect > 3
                    obj.sect = 3;
                    obj.fin = true;
                    break
                end
            end
            h_ref = obj.path.get_h(p_e, obj.sect);
            
            % Lateral position feedback
            err = obj.path.lat_err(p_e, obj.sect);
            h_del = clamp(-obj.h_gain * err, -obj.h_max, +obj.h_max);
            h = h_ref + h_del;
            
            % Linear autopilot
            u = obj.ctrl(obj.sect).update(obj.alt, h, x);
        end
    end
end