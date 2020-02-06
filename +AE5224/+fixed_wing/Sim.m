classdef Sim < AE5224.rigid_body.Sim
    %SIM Class for fixed-wing aircraft simulation
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public)
        function update(obj, de, da, dr, dp, w_e, p)
            %UPDATE(obj, de, da, dr, dp, w_e, p)
            %   Run one simulation loop
            %   
            %   Inputs:
            %   - de = Elevator angle [rad]
            %   - da = Aileron angle [rad]
            %   - dr = Rudder angle [rad]
            %   - dp = Prop throttle [0-1]
            %   - w_e = Wind velocity [Earth, m/s]
            %   - p = Air density [kg/m^3]
            
            % Imports
            import('AE5224.get_g');
            import('quat.Quat');
            
            % Air-speed transform
            Reb = Quat(obj.q_e).inv().mat_rot();
            [al, be, V] = obj.air_speed(Reb, w_e);
            
            % Angular velocity
            wx = obj.w_b(1);
            wy = obj.w_b(2);
            wz = obj.w_b(3);
            
            % Gravitational forces
            F_gz = obj.body.m * get_g();
            F_g = Reb * [0; 0; F_gz];
            
            % Longitudinal AFMs
            F_aer = 0.5 * p * V^2 * obj.body.S_wn;
            M_lon = F_aer * obj.body.c_wn;
            C_c = obj.body.c_wn / (2 * V);
            Fl = F_aer * obj.get_C_Fl(al, wy, de, C_c);
            Fd = F_aer * obj.get_C_Fd(al, wy, de, C_c);
            c_al = cos(al);
            s_al = sin(al);
            F_ax = +Fl*s_al - Fd*c_al;
            F_az = -Fl*c_al - Fd*s_al;
            M_ay = M_lon * obj.get_C_My(al, wy, de, C_c);
            
            % Lateral AFMs
            M_lat = F_aer * obj.body.b_wn;
            C_b = obj.body.b_wn / (2 * V);
            F_ay = F_aer * obj.get_C_Fy(be, wx, wz, da, dr, C_b);
            M_ax = M_lat * obj.get_C_Mx(be, wx, wz, da, dr, C_b);
            M_az = M_lat * obj.get_C_Mz(be, wx, wz, da, dr, C_b);
            
            % Prop FMs
            S_pr = obj.body.S_pr;
            C_pr = obj.body.C_pr;
            F_px = 0.5 * p * S_pr * C_pr * ((obj.body.k_v * dp)^2 - V^2);
            M_px = -obj.body.k_t * (obj.body.k_w * dp)^2;
            
            % Sum forces and moments
            F_a = [F_ax; F_ay; F_az];
            F_p = [F_px; 0; 0];
            F_b = F_g + F_a + F_p;
            M_a = [M_ax; M_ay; M_az];
            M_p = [M_px; 0; 0];
            M_b = M_a + M_p;
            
            % Update rigid body
            update@AE5224.rigid_body.Sim(obj, F_b, M_b);
        end
    end
    
    methods (Access = protected)
        function [al, be, V] = air_speed(obj, Reb, w_e)
            %[al, be, V] = AIR_SPEED(obj, Reb, w_e)
            %   Compute air-speed parameters
            %   
            %   Inputs:
            %   - Reb = Rotation matrix from Earth to Body
            %   - w_e = Wind velocity [Earth, m/s]
            %   
            %   Outputs:
            %   - al = Attack angle [rad]
            %   - be = Sideslip angle [rad]
            %   - V = Air-speed [m/s]
            v_a = Reb * (obj.v_e - w_e);
            vx = v_a(1);
            vy = v_a(2);
            vz = v_a(3);
            V = norm(v_a);
            al = atan(vz / vx);
            be = asin(vy / V);
        end
        
        function C_Fl = get_C_Fl(obj, al, wy, de, C_c)
            %C_Fl = GET_C_FL(obj, al, wy, de, C_c)
            %   Compute lift force coefficient
            %   
            %   Inputs:
            %   - al = Angle of attack [rad]
            %   - wy = Pitch velocity [rad/s]
            %   - de = Elevator angle [rad]
            %   - C_c = Chord coefficient [c/(2V)]
            %   
            %   Outputs:
            %   - C_Fl = Lift force coefficient
            C_Fl = ...
                obj.body.C_Fl_of + ...
                obj.body.C_Fl_al * al + ...
                obj.body.C_Fl_wy * C_c * wy + ...
                obj.body.C_Fl_de * de;
        end
        
        function C_Fd = get_C_Fd(obj, al, wy, de, C_c)
            %C_Fd = GET_C_FD(obj, al, wy, de, C_c)
            %   Compute drag force coefficient
            %   
            %   Inputs:
            %   - al = Angle of attack [rad]
            %   - wy = Pitch velocity [rad/s]
            %   - de = Elevator angle [rad]
            %   - C_c = Chord coefficient [c/(2V)]
            %   
            %   Outputs:
            %   - C_Fd = Drag force coefficient
            C_Fd = ...
                obj.body.C_Fd_of + ...
                obj.body.C_Fd_al * al + ...
                obj.body.C_Fd_wy * C_c * wy + ...
                obj.body.C_Fd_de * de;
        end
        
        function C_My = get_C_My(obj, al, wy, de, C_c)
            %C_My = GET_C_MY(obj, al, wy, de, C_c)
            %   Compute moment y coefficient
            %   
            %   Inputs:
            %   - al = Angle of attack [rad]
            %   - wy = Pitch velocity [rad/s]
            %   - de = Elevator angle [rad]
            %   - C_c = Chord coefficient [c/(2V)]
            %   
            %   Outputs:
            %   - C_My = Moment y coefficient
            C_My = ...
                obj.body.C_My_of + ...
                obj.body.C_My_al * al + ...
                obj.body.C_My_wy * C_c * wy + ...
                obj.body.C_My_de * de;
        end
        
        function C_Fy = get_C_Fy(obj, be, wx, wz, da, dr, C_b)
            %C_Fy = GET_C_FY(obj, be, wx, wz, da, dr, C_b)
            %   Compute side force coefficient
            %   
            %   Inputs:
            %   - be = Sideslip angle [rad]
            %   - wx = Roll velocity [rad/s]
            %   - wz = Yaw velocity [rad/s]
            %   - da = Aileron angle [rad]
            %   - dr = Rudder angle [rad]
            %   - C_b = Wingspan coefficient [b/2V]
            %   
            %   Outputs:
            %   - C_Fy = Side force coefficient
            C_Fy = ...
                obj.body.C_Fy_of + ...
                obj.body.C_Fy_be * be + ...
                obj.body.C_Fy_wx * C_b * wx + ...
                obj.body.C_Fy_wz * C_b * wz + ...
                obj.body.C_Fy_da * da + ...
                obj.body.C_Fy_dr * dr;
        end
        
        function C_Mx = get_C_Mx(obj, be, wx, wz, da, dr, C_b)
            %C_Mx = GET_C_MX(obj, be, wx, wz, da, dr, C_b)
            %   Compute moment x coefficient
            %   
            %   Inputs:
            %   - be = Sideslip angle [rad]
            %   - wx = Roll velocity [rad/s]
            %   - wz = Yaw velocity [rad/s]
            %   - da = Aileron angle [rad]
            %   - dr = Rudder angle [rad]
            %   - C_b = Wingspan coefficient [b/2V]
            %   
            %   Outputs:
            %   - C_Mx = Moment x coefficient
            C_Mx = ...
                obj.body.C_Mx_of + ...
                obj.body.C_Mx_be * be + ...
                obj.body.C_Mx_wx * C_b * wx + ...
                obj.body.C_Mx_wz * C_b * wz + ...
                obj.body.C_Mx_da * da + ...
                obj.body.C_Mx_dr * dr;
        end
        
        function C_Mz = get_C_Mz(obj, be, wx, wz, da, dr, C_b)
            %C_Mz = GET_C_MZ(obj, be, wx, wz, da, dr, C_b)
            %   Compute moment z coefficient
            %   
            %   Inputs:
            %   - be = Sideslip angle [rad]
            %   - wx = Roll velocity [rad/s]
            %   - wz = Yaw velocity [rad/s]
            %   - da = Aileron angle [rad]
            %   - dr = Rudder angle [rad]
            %   - C_b = Wingspan coefficient [b/2V]
            %   
            %   Outputs:
            %   - C_Mz = Moment z coefficient
            C_Mz = ...
                obj.body.C_Mz_of + ...
                obj.body.C_Mz_be * be + ...
                obj.body.C_Mz_wx * C_b * wx + ...
                obj.body.C_Mz_wz * C_b * wz + ...
                obj.body.C_Mz_da * da + ...
                obj.body.C_Mz_dr * dr;
        end
    end
end