classdef Linear < handle
    %LINEAR Class for linearized aircraft control
    
    properties (SetAccess = protected)
        A_lon;      % Lon state matrix
        B_lon;      % Lon input matrix
        x_lon_st;   % Lon trim state
        u_lon_st;   % Lon trim input
        K_lon;      % Lon gain matrix
        A_lat;      % Lat state matrix
        B_lat;      % Lat input matrix
        x_lat_st;   % Lat trim state
        u_lat_st;   % Lat trim input
        K_lat;      % Lat gain matrix
    end
    
    methods (Access = public)
        function obj = Linear(model, x_st, u_st)
            %obj = LINEAR(model, x_st, u_st, verbose)
            %   Construct linear controller
            %   - model = Aircraft model [AE5224.fixed_wing.Model]
            %   - x_st = Trim states [p_e; q_e; v_e; w_b]
            %   - u_st = Trim controls [d_e; d_a; d_r; d_p]
            %   - verbose = Print flag [logical, def = true]
            
            % Imports
            import('AE5224.fixed_wing.control.linear.lon.lon_lqr');
            import('AE5224.fixed_wing.control.linear.lat.lat_lqr');
            import('AE5224.fixed_wing.control.make_lqr');
            import('AE5224.rigid_body.Model.unpack_x');
            import('quat.Quat');

            % Construction
            [obj.A_lon, obj.B_lon, obj.x_lon_st, obj.u_lon_st, obj.K_lon] = ...
                lon_lqr(model, x_st, u_st);
            [obj.A_lat, obj.B_lat, obj.x_lat_st, obj.u_lat_st, obj.K_lat] = ...
                lat_lqr(model, x_st, u_st);
        end
        
        function u = update(obj, alt, th_z, x)
            %u = UPDATE(obj, alt, th_z, x, t)
            %   Update control output with new state
            %   - alt = Altitude [m]
            %   - th_z = Heading cmd [m]
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - u = Controls [d_e; d_a; d_r; d_p]
            
            % Imports
            import('AE5224.fixed_wing.control.linear.lon.x_to_xlon');
            import('AE5224.fixed_wing.control.linear.lat.x_to_xlat');
            import('AE5224.fixed_wing.control.linear.lon.ulon_to_u');
            import('AE5224.fixed_wing.control.linear.lat.ulat_to_u');
            import('controls.wrap');
            
            % Trim setpoints
            x_lon_st_ = obj.x_lon_st;
            x_lat_st_ = obj.x_lat_st;
            x_lon_st_(5) = alt;
            x_lat_st_(5) = th_z;
            
            % Trim errors
            dx_lon = x_to_xlon(x) - x_lon_st_;
            dx_lat = x_to_xlat(x) - x_lat_st_;
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