classdef Mag < AE5224.sensors.Sensor
    %MAG Magnetometer simulator
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public)
        function obj = Mag(cov_b)
            %obj = MAG(cov_b)
            %   Construct magnetometer
            %   
            %   Inputs:
            %   - cov_b = Field cov matrix [uT^2]
            
            % Default args
            if nargin < 1
                cov_b = diag(repmat(10^2, 3, 1)); % TODO get from sensor
            end
            
            % Construction
            obj@AE5224.sensors.Sensor(cov_b);
        end
        
        function z = measure(obj, x)
            %z = MEASURE(obj, x)
            %   Simulate magnetometer reading
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   
            %   Outputs:
            %   - z = Observation [b_bx; b_by; b_bz]
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            import('AE5224.const.get_b');
            import('quat.Quat');
            
            % Function
            [~, q_e, ~, ~] = unpack_x(x);
            b_e = get_b();
            b_b = Quat(q_e).inv().rotate(b_e);
            z = mvnrnd(b_b, obj.cov_z).';
        end
    end
end