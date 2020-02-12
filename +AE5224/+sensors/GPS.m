classdef GPS < AE5224.sensors.Sensor
    %GPS Class for simulating GPS measurements
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        cov_p;  % Position cov matrix [m^2]
        cov_v;  % Velocity cov matrix [(m/s)^2]
    end
    
    methods (Access = public, Static)
        function z = pack_z(p_e, v_e)
            %z = PACK_Z(p_e, v_e)
            %   Make measurement vector from components
            %   - p_e = Position Earth [m]
            %   - v_e = Velocity Earth [m/s]
            %   - z = Measurement vector
            z = [p_e; v_e];
        end
        
        function [p_e, v_e] = unpack_z(z)
            %[p_e, v_e] = UNPACK_Z(z)
            %   Get components from measurement vector
            %   - z = Measurement vector
            %   - p_e = Position Earth [m]
            %   - v_e = Velocity Earth [m/s]
            p_e = z(1:3);
            v_e = z(4:6);
        end
    end
    
    methods (Access = public)
        function obj = GPS(cov_p, cov_v)
            %obj = GPS(cov_p, cov_v)
            %   Construct GPS simulator
            %   
            %   Inputs:
            %   - cov_p = Position cov matrix [m^2]
            %   - cov_v = Velocity cov matrix [(m/s)^2]
            %   
            %   Default values com from textbook
            
            % Default args
            if nargin < 1, cov_p = diag([6.60, 6.60, 9.20]); end
            if nargin < 2, cov_v = diag([0.05, 0.05, 0.05]); end
            
            % Superconstructor
            zer = zeros(3);
            cov_z = [cov_p, zer; zer, cov_v];
            obj@AE5224.sensors.Sensor(cov_z);
            
            % Individual covs
            obj.cov_p = cov_p;
            obj.cov_v = cov_v;
        end
        
        function z = measure(obj, x)
            %z = MEASURE(obj, x)
            %   Simulate sensor reading
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   
            %   Outputs:
            %   - z = Observation vector [p_e; v_e]
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            
            % Function
            [p_e, ~, v_e, ~] = unpack_x(x);
            z = obj.pack_z(p_e, v_e);
            z = mvnrnd(z, obj.cov_z).';
        end
    end
end