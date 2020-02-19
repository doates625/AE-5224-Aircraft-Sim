classdef Controller < AE5224.control.Controller
    %CONTROLLER Class for linearized fixed-wing aircraft control
    
    properties (SetAccess = protected)
        x_lon_st;   % Trim lon state
        x_lat_st;   % Trim lat state
        u_lon_st;   % Trim lon inputs
        u_lat_st;   % Trim lat inputs
        K_lon;      % Lon feedback matrix
        K_lat;      % Lat feedback matrix
    end
    
    methods (Access = public)
        function obj = Controller(model, x_st, u_st, verbose)
            %obj = CONTROLLER(model, x_st, u_st, verbose)
            %   Construct fixed-wing controller
            %   - model = Aircraft model [AE5224.fixed_wing.Model]
            %   - x_st = Trim states [p_e; q_e; v_e; w_b]
            %   - u_st = Trim controls [d_e; d_a; d_r; d_p]
            %   - verbose = Print flag [logical, def = true]
            
            % Imports
            import('AE5224.fixed_wing.control.make_lqr');
            import('AE5224.rigid_body.Model.unpack_x');
            import('quat.Quat');
            
            % Default args
            if nargin < 4, verbose = true; end
            
            % Superconstructor
            obj@AE5224.control.Controller(model.u_min, model.u_max);
            
            % Subsystem controllers
            [x_lon_st_, obj.u_lon_st, obj.K_lon] = ...
                AE5224.fixed_wing.control.lon.make_lqr(...
                    model, x_st, u_st, verbose);
            [x_lat_st_, obj.u_lat_st, obj.K_lat] = ...
                AE5224.fixed_wing.control.lat.make_lqr(...
                    model, x_st, u_st, verbose);
            
            % Non-constant trims
            [~, q_e, v_e, w_b] = unpack_x(x_st);
            w_e = Quat(q_e).rotate(w_b);
            obj.x_lon_st = @(t) [...
                x_lon_st_(1:4);
                x_lon_st_(5) - v_e(3)*t;
            ];
            obj.x_lat_st = @(t) [...
                x_lat_st_(1:4);
                controls.wrap(x_lat_st_(5) + w_e(3)*t, -pi, +pi);
            ];
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x, t)
            %u = UPDATE_(obj, x, t)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - t = Current time [s]
            %   - u = Unsurated controls [d_e; d_a; d_r; d_p]
            
            % Imports
            import('AE5224.fixed_wing.control.lon.x_to_xlon');
            import('AE5224.fixed_wing.control.lat.x_to_xlat');
            import('AE5224.fixed_wing.control.lon.ulon_to_u');
            import('AE5224.fixed_wing.control.lat.ulat_to_u');
            import('controls.wrap');
            
            % Trim errors
            dx_lon = x_to_xlon(x) - obj.x_lon_st(t);
            dx_lat = x_to_xlat(x) - obj.x_lat_st(t);
            dx_lat(5) = wrap(dx_lat(5), -pi, +pi);
            
            % Linearized controls
            u_lon = obj.u_lon_st - obj.K_lon * dx_lon;
            u_lat = obj.u_lat_st - obj.K_lat * dx_lat;
            
            % Combine controls
            u_lon = ulon_to_u(u_lon, zeros(4, 1));
            u_lat = ulat_to_u(u_lat, zeros(4, 1));
            u = u_lon + u_lat;
        end
    end
end