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
        function obj = Controller(model, x_st, u_st)
            %obj = CONTROLLER(model, trim)
            %   Construct fixed-wing controller
            %   - model = Aircraft model [AE5224.fixed_wing.Model]
            %   - x_st = Trim states [p_e; q_e; v_e; w_b]
            %   - u_st = Trim controls [d_e; d_a; d_r; d_p]
            
            % Imports
            import('AE5224.fixed_wing.control.make_lqr');
            
            % Superconstructor
            obj@AE5224.control.Controller(model.u_min, model.u_max);
            
            % Subsystem controllers
            [obj.x_lon_st, obj.u_lon_st, obj.K_lon] = ...
                AE5224.fixed_wing.control.lon.make_lqr(model, x_st, u_st);
            [obj.x_lat_st, obj.u_lat_st, obj.K_lat] = ...
                AE5224.fixed_wing.control.lat.make_lqr(model, x_st, u_st);
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x)
            %u = UPDATE_(obj, x)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - u = Unsurated controls [d_e; d_a; d_r; d_p]
            
            % Imports
            import('AE5224.fixed_wing.control.lon.x_to_xlon');
            import('AE5224.fixed_wing.control.lat.x_to_xlat');
            import('AE5224.fixed_wing.control.lon.ulon_to_u');
            import('AE5224.fixed_wing.control.lat.ulat_to_u');
            
            % Linearized control
            u_lon = obj.u_lon_st - obj.K_lon * (x_to_xlon(x) - obj.x_lon_st);
            u_lat = obj.u_lat_st - obj.K_lat * (x_to_xlat(x) - obj.x_lat_st);
            
            % Comine controls
            u_lon = ulon_to_u(u_lon, zeros(4, 1));
            u_lat = ulat_to_u(u_lat, zeros(4, 1));
            u = u_lon + u_lat;
        end
    end
end