classdef Model < AE5224.rigid_body.Model
    %MODEL Class for general aircraft models
    
    properties (SetAccess = protected)
        u_min;  % Min controls
        u_max;  % Max controls
    end
    
    methods (Access = public)
        function obj = Model(m, I_b, u_min, u_max)
            %obj = MODEL(m, I_b)
            %   Construct rigid body model
            %   
            %   Inputs:
            %   - m = Mass [kg]
            %   - I_b = Body-fixed inertia [kg*m^2]
            %   - u_min = Min controls
            %   - u_max = Max controls
            obj@AE5224.rigid_body.Model(m, I_b);
            obj.u_min = u_min;
            obj.u_max = u_max;
        end
    end
end