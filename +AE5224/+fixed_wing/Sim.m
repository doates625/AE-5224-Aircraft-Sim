classdef Sim < AE5224.rigid_body.Sim
    %SIM Class for fixed-wing aircraft simulation
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public)
        function dx_dt = dynamics(obj, x, u)
            %dx_dt = DYNAMICS(obj, x, u)
            %   Compute state time derivative
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   - u = Input vector [d_e; d_a; d_r; d_p]
            %   
            %   Input vector:
            %   - d_e = Elevator angle [rad]
            %   - d_a = Aileron angle [rad]
            %   - d_r = Rudder angle [rad]
            %   - d_p = Prop throttle [0-1]
            %   
            %   Outputs:
            %   - dx_dt = State time derivative
            
            % Imports
            import('AE5224.get_g');
            import('AE5224.get_p');
            import('quat.Quat');
            
            % Constants
            g = get_g();
            p = get_p();
            
            % Unpack states and controls
            [~, q_e, v_e, w_b] = obj.unpack(x);
            w_bx = w_b(1);
            w_by = w_b(2);
            w_bz = w_b(3);
            d_e = u(1);
            d_a = u(2);
            d_r = u(3);
            d_p = u(4);
            
            % Air-speed transform
            R_eb = Quat(q_e).inv().mat_rot();
            v_a = R_eb * v_e;
            v_ax = v_a(1);
            v_ay = v_a(2);
            v_az = v_a(3);
            V = norm(v_a);
            al = atan(v_az / v_ax);
            be = asin(v_ay / V);
            
            % Longitudinal AFMs
            F_air = 0.5 * p * V^2 * obj.body.S_wn;
            M_lon = F_air * obj.body.c_wn;
            C_c = obj.body.c_wn / (2 * V);
            C_Fl = ...
                obj.body.C_Fl_of + ...
                obj.body.C_Fl_al * al + ...
                obj.body.C_Fl_wy * C_c * w_by + ...
                obj.body.C_Fl_de * d_e;
            C_Fd = ...
                obj.body.C_Fd_of + ...
                obj.body.C_Fd_al * al + ...
                obj.body.C_Fd_wy * C_c * w_by + ...
                obj.body.C_Fd_de * d_e;
            C_My = ...
                obj.body.C_My_of + ...
                obj.body.C_My_al * al + ...
                obj.body.C_My_wy * C_c * w_by + ...
                obj.body.C_My_de * d_e;
            Fl = F_air * C_Fl;
            Fd = F_air * C_Fd;
            c_al = cos(al);
            s_al = sin(al);
            F_ax = +Fl*s_al - Fd*c_al;
            F_az = -Fl*c_al - Fd*s_al;
            M_ay = M_lon * C_My;
            
            % Lateral AFMs
            M_lat = F_air * obj.body.b_wn;
            C_b = obj.body.b_wn / (2 * V);
            C_Fy = ...
                obj.body.C_Fy_of + ...
                obj.body.C_Fy_be * be + ...
                obj.body.C_Fy_wx * C_b * w_bx + ...
                obj.body.C_Fy_wz * C_b * w_bz + ...
                obj.body.C_Fy_da * d_a + ...
                obj.body.C_Fy_dr * d_r;
            C_Mx = ...
                obj.body.C_Mx_of + ...
                obj.body.C_Mx_be * be + ...
                obj.body.C_Mx_wx * C_b * w_bx + ...
                obj.body.C_Mx_wz * C_b * w_bz + ...
                obj.body.C_Mx_da * d_a + ...
                obj.body.C_Mx_dr * d_r;
            C_Mz = ...
                obj.body.C_Mz_of + ...
                obj.body.C_Mz_be * be + ...
                obj.body.C_Mz_wx * C_b * w_bx + ...
                obj.body.C_Mz_wz * C_b * w_bz + ...
                obj.body.C_Mz_da * d_a + ...
                obj.body.C_Mz_dr * d_r;
            F_ay = F_air * C_Fy;
            M_ax = M_lat * C_Mx;
            M_az = M_lat * C_Mz;
            
            % Prop FMs
            S_pr = obj.body.S_pr;
            C_pr = obj.body.C_pr;
            F_px = 0.5 * p * S_pr * C_pr * ((obj.body.k_v * d_p)^2 - V^2);
            M_px = -obj.body.k_t * (obj.body.k_w * d_p)^2;
            
            % Gravitational forces
            F_gz = obj.body.m * g;
            F_g = R_eb * [0; 0; F_gz];
            
            % Sum forces and moments
            F_a = [F_ax; F_ay; F_az];
            F_p = [F_px; 0; 0];
            F_b = F_a + F_p + F_g;
            M_a = [M_ax; M_ay; M_az];
            M_p = [M_px; 0; 0];
            M_b = M_a + M_p;
            
            % Rigid body dynamics
            dx_dt = dynamics@AE5224.rigid_body.Sim(obj, x, [F_b; M_b]);
        end
    end
end