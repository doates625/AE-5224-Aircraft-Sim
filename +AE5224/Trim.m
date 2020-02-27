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
        a_e;    % Init Earth accel [m/s]
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
            obj.a_e = obj.v_e(1) * obj.w_e(3) * [0; 1; 0];
        end
        
        function c = eq(t1, t2)
            %EQ(t1, t2) Test trim equality
            c = isequal(...
                [t1.V, t1.R, t1.gam, t1.h], ...
                [t2.V, t2.R, t2.gam, t2.h]);
        end
    end
end