classdef Dubins < AE5224.quad_rotor.control.Controller
    %DUBINS Quadrotor dubins path tracking controller
    
    properties (Access = protected)
        path;   % Dubins path [AE5224.dubins.path.Path]
        vel;    % Airspeed [m/s]
        alt;    % Altitude [m]
    end
    
    methods (Access = public)
        function obj = Dubins(model, path, vel, alt, del_t)
            %obj = DUBINS(model, path, vel, alt, del_t)
            %   Construct Dubins tracking controller
            %   - model = Quadrotor model [AE5224.quad_rotor.Model]
            % 	- path = Dubins path [AE5224.dubins.path.Path]
            %   - vel = Airspeed [m/s]
            %   - alt = Altitude [m]
            %   - del_t = Time delta [s]
            obj@AE5224.quad_rotor.control.Controller(model, del_t);
            obj.path = path;
            obj.vel = vel;
            obj.alt = alt;
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x, t)
            %u = UPDATE(obj, x, t)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - t = Current time [s]
            %   - u = Unsurated control vector
            
            % Ppsition and heading
            d = obj.vel * t;
            p_e_ref = [0; 0; -obj.alt];
            [p_e_ref(1:2), th_z_ref, sect] = obj.path.get(d);
            
            % Acceleration
            switch sect
                case 1
                    acc_c = obj.vel^2 / obj.path.rad * obj.path.sgn_a();
                case 2
                    acc_c = 0;
                case 3
                    acc_c = obj.vel^2 / obj.path.rad * obj.path.sgn_b();
            end
            a_e_ref = acc_c * [-sin(th_z_ref); +cos(th_z_ref); 0];
            
            % Control signals
            u = update_@AE5224.quad_rotor.control.Controller(...
                obj, x, p_e_ref, a_e_ref, th_z_ref);
        end
    end
end