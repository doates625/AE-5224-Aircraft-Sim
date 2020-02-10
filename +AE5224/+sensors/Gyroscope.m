classdef Gyroscope < AE5224.sensors.Sensor
    %GYROSCOPE Gyroscope simulator
    
    methods (Access = public)
        function obj = Gyroscope(f_s, n_d)
            %obj = GYROSCOPE(f_s, n_d)
            %   Construct gyroscope simulator
            %   
            %   Inputs:
            %   - f_s = Sample rate [Hz]
            %   - n_d = Noise density [(rad/s)/sqrt(Hz), def = 2.62e-04]
            
            % Default args
            if nargin < 2, n_d = 2.62e-04; end
            
            % Construction
            cov_w = diag(repmat(n_d^2 * f_s, 3, 1));
            obj@AE5224.sensors.Sensor(cov_w);
        end
        
        function z = measure(obj, x)
            %z = MEASURE(obj, x)
            %   Simulate accelerometer reading
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   
            %   Outputs:
            %   - z = Observation [w_bx; w_by; w_bz]
            
            % Imports
            import('AE5224.rigid_body.Body.unpack');
            
            % Function
            [~, ~, ~, w_b] = unpack(x);
            z = mvnrnd(w_b, obj.cov_z).';
        end
    end
end