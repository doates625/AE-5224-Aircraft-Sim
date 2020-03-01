classdef Trim < AE5224.control.Controller
    %TRIM Fixed-wing trim tracking controller
    
    properties (Access = protected)
        alt_f;  % Altitude func [function_handle]
        th_z_f; % Heading func [function_handle]
        ctrl;   % Controller [AE5224.fixed_wing.control.linear.Linear]
    end
    
    methods (Access = public)
        function obj = Trim(model, x_st, u_st)
            %obj = TRIM(model, x_st, u_st)
            %   Construct trim tracking controller
            %   - model = Fixed-wing model [AE5224.fixed_wing.Model]
            % 	- x_st = Trim state vector [p_e; q_e; v_e; w_b]
            %   - u_st = Trim control vector [d_e; d_a; d_r; d_p]
            %   - file = Load and save file [char]
            
            % Imports
            import('AE5224.fixed_wing.control.linear.lon.x_to_xlon');
            import('AE5224.fixed_wing.control.linear.lat.x_to_xlat');
            import('AE5224.fixed_wing.control.linear.Linear');
            import('AE5224.rigid_body.Model.unpack_x');
            import('quat.Quat');
            
            % Construction
            obj@AE5224.control.Controller(model);
            x_lon = x_to_xlon(x_st);
            x_lat = x_to_xlat(x_st);
            [~, q_e, v_e, w_b] = unpack_x(x_st);
            w_e = Quat(q_e).rotate(w_b);
            obj.ctrl = Linear(model, x_st, u_st);
            obj.alt_f = @(t) x_lon(5) - v_e(3) * t;
            obj.th_z_f = @(t) x_lat(5) + w_e(3) * t;
        end
        
        function disp(obj)
            %DISP(obj) Display controller info
            disp(obj.ctrl);
        end
    end
    
    methods (Access = protected)
        function u = update_(obj, x, t)
            %u = UPDATE(obj, x, t)
            %   Update control output with new state
            %   - x = State [p_e; q_e; v_e; w_b]
            %   - t = Current time [s]
            %   - u = Unsurated control vector
            alt = obj.alt_f(t);
            th_z = obj.th_z_f(t);
            u = obj.ctrl.update(alt, th_z, x);
        end
    end
end