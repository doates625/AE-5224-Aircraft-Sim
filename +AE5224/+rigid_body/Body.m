classdef Body < handle
    %BODY Class for 3D rigid body models
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        m;      % Mass [kg]
        I_b;    % Body-fixed inertia [kg*m^2]
    end
    
    methods (Access = public)
        function obj = Body(m, I_b)
            %obj = BODY(m, I_b)
            %   Construct rigid body model
            %   
            %   Inputs:
            %   - m = Mass [kg]
            %   - I_b = Body-fixed inertia [kg*m^2]
            obj.m = m;
            obj.I_b = I_b;
        end
    end
end