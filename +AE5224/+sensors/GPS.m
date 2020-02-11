classdef GPS < AE5224.sensors.Sensor
    %GPS Class for simulating GPS measurements
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public)
        function obj = GPS(cov_p, cov_v)
            %obj = GPS(cov_p, cov_v)
            %   Construct GPS simulator
            %   
            %   Inputs:
            %   - cov_p = Position cov matrix
            %   - cov_v = Velocity cov matrix
            %   
            %   Default values com from textbook
            
            % Default args
            if nargin < 1, cov_p = diag([6.60, 6.60, 9.20]); end
            if nargin < 2, cov_v = diag([0.05, 0.05, 0.05]); end
            
            % Superconstructor
            zer = zeros(3);
            cov_z = [cov_p, zer; zer, cov_v];
            obj@AE5224.sensors.Sensor(cov_z);
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
            z = mvnrnd([p_e; v_e], obj.cov_z).';
        end
    end
end