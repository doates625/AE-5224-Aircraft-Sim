classdef Model < AE5224.rigid_body.Model
    %MODEL Class for fixed-wing aircraft models
    %   
    %   Input vector u:
    %   - d_e = Elevator angle [rad]
    %   - d_a = Aileron angle [rad]
    %   - d_r = Rudder angle [rad]
    %   - d_p = Prop throttle [0-1]
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        % Dimensional constants
        S_wn;       % Wing planform area [m^2]
        c_wn;       % Wing mean chord [m]
        b_wn;       % Wing span [m]
        
        % Prop constants
        S_pr;       % Prop sweep area [m^2]
        C_pr;       % Prop aerodynamic coefficient (AC)
        k_v;        % Prop airspeed constant [m/s]
        k_w;        % Prop angle rate contant [rad/s]
        k_t;        % Prop torque constant [N*m/(rad/s)^2]
        
        % Lift force ACs
        C_Fl_of;    % Offset
        C_Fl_al;    % Attack angle
        C_Fl_wy;    % Angle rate y
        C_Fl_de;    % Elevator
        
        % Drag force ACs
        C_Fd_of;    % Offset
        C_Fd_al;    % Attack angle
        C_Fd_wy;    % Angle rate y
        C_Fd_de;    % Elevator
        
        % Moment y ACs
        C_My_of;    % Offset
        C_My_al;    % Attack angle
        C_My_wy;    % Angle rate y
        C_My_de;    % Elevator
        
        % Side force ACs
        C_Fy_of;    % Offset
        C_Fy_be;    % Sideslip angle
        C_Fy_wx;    % Angle rate x
        C_Fy_wz;    % Angle rate z
        C_Fy_da;    % Aileron
        C_Fy_dr;    % Rudder
        
        % Moment x ACs
        C_Mx_of;    % Offset
        C_Mx_be;    % Sideslip angle
        C_Mx_wx;    % Angle rate x
        C_Mx_wz;    % Angle rate z
        C_Mx_da;    % Aileron
        C_Mx_dr;    % Rudder
        
        % Moment z ACs
        C_Mz_of;    % Offset
        C_Mz_be;    % Sideslip angle
        C_Mz_wx;    % Angle rate x
        C_Mz_wz;    % Angle rate z
        C_Mz_da;    % Aileron
        C_Mz_dr;    % Rudder
    end
    
    methods (Access = public)
        function obj = Model()
            %obj = MODEL()
            %   Create fixed-wing aircraft model
            %   Uses parameters from Aerosonde UAV
            
            % Inertial constants
            m = 13.5;               % Mass [kg]
            I_xx = 0.8244;          % Inertia xx [kg*m^2]
            I_yy = 1.135;           % Inertia yy [kg*m^2]
            I_zz = 1.759;           % Inertia zz [kg*m^2]
            I_xy = 0;               % Inertia xy [kg*m^2]
            I_xz = 0.1204;          % Inertia xz [kg*m^2]
            I_yz = 0;               % Inertia yz [kg*m^2]
            I_b = [...
                [I_xx, I_xy, I_xz]; ...
                [I_xy, I_yy, I_yz]; ...
                [I_xz, I_yz, I_zz]];
            obj@AE5224.rigid_body.Model(m, I_b);
            
            % Dimensional constants
            obj.S_wn = 0.55;        % Wing planform area [m^2]
            obj.c_wn = 0.18994;     % Wing mean chord [m]
            obj.b_wn = 2.8956;      % Wing span [m]

            % Prop constants
            obj.S_pr = 0.2027;      % Prop sweep area [m^2]
            obj.C_pr = 1.0;         % Prop aerodynamic coefficient (AC)
            obj.k_v = 80;           % Prop airspeed constant [m/s]
            obj.k_w = 0;            % Prop angle rate contant [rad/s]
            obj.k_t = 0;            % Prop torque constant [N*m/(rad/s)^2]

            % Lift force ACs
            obj.C_Fl_of = 0.28;     % Offset
            obj.C_Fl_al = 3.45;     % Attack angle
            obj.C_Fl_wy = 0;        % Angle rate y
            obj.C_Fl_de = -0.36;    % Elevator

            % Drag force ACs
            obj.C_Fd_of = 0.03;     % Offset
            obj.C_Fd_al = 0.30;     % Attack angle
            obj.C_Fd_wy = 0;        % Angle rate y
            obj.C_Fd_de = 0;        % Elevator

            % Moment y ACs
            obj.C_My_of = -0.02338; % Offset
            obj.C_My_al = -0.38;    % Attack angle
            obj.C_My_wy = -3.6;     % Angle rate y
            obj.C_My_de = -0.5;     % Elevator

            % Side force ACs
            obj.C_Fy_of = 0;        % Offset
            obj.C_Fy_be = -0.98;    % Sideslip angle
            obj.C_Fy_wx = 0;        % Angle rate x
            obj.C_Fy_wz = 0;        % Angle rate z
            obj.C_Fy_da = 0;        % Aileron
            obj.C_Fy_dr = -0.17;    % Rudder

            % Moment x ACs
            obj.C_Mx_of = 0;        % Offset
            obj.C_Mx_be = -0.12;    % Sideslip angle
            obj.C_Mx_wx = -0.26;    % Angle rate x
            obj.C_Mx_wz = 0.14;     % Angle rate z
            obj.C_Mx_da = 0.08;     % Aileron
            obj.C_Mx_dr = 0.105;    % Rudder

            % Moment z ACs
            obj.C_Mz_of = 0;        % Offset
            obj.C_Mz_be = 0.25;     % Sideslip angle
            obj.C_Mz_wx = 0.022;    % Angle rate x
            obj.C_Mz_wz = -0.35;    % Angle rate z
            obj.C_Mz_da = 0.08;     % Aileron
            obj.C_Mz_dr = -0.032;   % Rudder
        end
        
        function [F_b, M_b] = forces(obj, x, u)
            %[F_b, M_b] = FORCES(obj, x, u)
            %   Compute body-frame net forces and moments
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   - u = Input vector [d_e; d_a; d_r; d_p]
            %   
            %   Outputs:
            %   - F_b = Net force vector [N]
            %   - M_b = Net moment vector [N*m]
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            import('AE5224.const.get_g');
            import('AE5224.const.get_p');
            import('quat.Quat');
            
            % Constants
            g = get_g();
            p = get_p();
            
            % Unpack states and controls
            [~, q_e, v_e, w_b] = unpack_x(x);
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
            V_a = norm(v_a);
            al = atan(v_az / v_ax);
            be = asin(v_ay / V_a);
            
            % Longitudinal AFMs
            F_air = 0.5 * p * V_a^2 * obj.S_wn;
            M_lon = F_air * obj.c_wn;
            C_c = obj.c_wn / (2 * V_a);
            C_Fl = ...
                obj.C_Fl_of + ...
                obj.C_Fl_al * al + ...
                obj.C_Fl_wy * C_c * w_by + ...
                obj.C_Fl_de * d_e;
            C_Fd = ...
                obj.C_Fd_of + ...
                obj.C_Fd_al * al + ...
                obj.C_Fd_wy * C_c * w_by + ...
                obj.C_Fd_de * d_e;
            C_My = ...
                obj.C_My_of + ...
                obj.C_My_al * al + ...
                obj.C_My_wy * C_c * w_by + ...
                obj.C_My_de * d_e;
            Fl = F_air * C_Fl;
            Fd = F_air * C_Fd;
            c_al = cos(al);
            s_al = sin(al);
            F_ax = +Fl*s_al - Fd*c_al;
            F_az = -Fl*c_al - Fd*s_al;
            M_ay = M_lon * C_My;
            
            % Lateral AFMs
            M_lat = F_air * obj.b_wn;
            C_b = obj.b_wn / (2 * V_a);
            C_Fy = ...
                obj.C_Fy_of + ...
                obj.C_Fy_be * be + ...
                obj.C_Fy_wx * C_b * w_bx + ...
                obj.C_Fy_wz * C_b * w_bz + ...
                obj.C_Fy_da * d_a + ...
                obj.C_Fy_dr * d_r;
            C_Mx = ...
                obj.C_Mx_of + ...
                obj.C_Mx_be * be + ...
                obj.C_Mx_wx * C_b * w_bx + ...
                obj.C_Mx_wz * C_b * w_bz + ...
                obj.C_Mx_da * d_a + ...
                obj.C_Mx_dr * d_r;
            C_Mz = ...
                obj.C_Mz_of + ...
                obj.C_Mz_be * be + ...
                obj.C_Mz_wx * C_b * w_bx + ...
                obj.C_Mz_wz * C_b * w_bz + ...
                obj.C_Mz_da * d_a + ...
                obj.C_Mz_dr * d_r;
            F_ay = F_air * C_Fy;
            M_ax = M_lat * C_Mx;
            M_az = M_lat * C_Mz;
            
            % Prop FMs
            V_p = obj.k_v * d_p;
            F_px = 0.5 * p * obj.S_pr * obj.C_pr * (V_p^2 - V_a^2);
            M_px = -obj.k_t * (obj.k_w * d_p)^2;
            
            % Gravitational forces
            F_gz = obj.m * g;
            F_g = R_eb * [0; 0; F_gz];
            
            % Sum forces and moments
            F_a = [F_ax; F_ay; F_az];
            F_p = [F_px; 0; 0];
            F_b = F_a + F_p + F_g;
            M_a = [M_ax; M_ay; M_az];
            M_p = [M_px; 0; 0];
            M_b = M_a + M_p;
        end
    end
end