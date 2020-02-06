classdef Body < AE5224.rigid_body.Body
    %BODY Class for fixed-wing aircraft rigid body models
    %   
    %   Modeling assumptions:
    %   - Linearized aerodynamic coefficients
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
        function obj = Body()
            %obj = BODY()
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
            obj@AE5224.rigid_body.Body(m, I_b);
            
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
    end
end