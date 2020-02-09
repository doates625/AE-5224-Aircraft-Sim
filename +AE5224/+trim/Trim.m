classdef Trim < handle
    %TRIM Class for trim condition data
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        V;      % Airspeed [m/s]
        R;      % Turn radius [m]
        gam;    % Climb angle [rad]
        h;      % Init altitude [m]
        p_e;    % Init Earth position [m]
        v_e;    % Init Earth velocity [m/s]
        w_e;    % Init Earth angle rate [rad/s]
    end
    
    methods (Access = public)
        function obj = Trim(V, R, gam, h)
            %obj = TRIM(V, R, gam, h)
            %   Construct trim condition
            %   
            %   Inputs:
            %   - V = Airspeed [m/s]
            %   - R = Turn radius [m]
            %   - gam = Climb angle [rad]
            %   - h = Init altitude [m]
            obj.V = V;
            obj.R = R;
            obj.gam = gam;
            obj.h = h;
            obj.p_e = [0; 0; -h];
            c_gam = cos(gam);
            s_gam = sin(gam);
            obj.v_e = V * [c_gam; 0; -s_gam];
            obj.w_e = V * c_gam / R * [0; 0; 1];
        end
        
        function F_c = get_F_c(obj, m)
            %F_c = GET_F_C(obj, body)
            %   Get trim centripetal force
            %   
            %   Inputs:
            %   - m = Body mass [kg]
            %   
            %   Outputs:
            %   - F_c = Centripetal force [N]
            F_c = m * obj.V^2 / obj.R;
        end
    end
end