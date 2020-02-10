classdef Magnetometer < AE5224.sensors.Sensor
    %MAGNETOMETER Magnetometer simulator
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = public, Constant)
        b_e = [50; 0; 0];   % Earth-fixed magnetic field [uT]
    end
    
    methods (Access = public)
        function obj = Magnetometer(cov_b)
            %obj = MAGNETOMETER(cov_b)
            %   Construct magnetometer simulator
            %   
            %   Inputs:
            %   - cov_b = Field cov matrix [uT^2]
            
            % Default args
            if nargin < 1
                cov_b = diag(repmat(10, 3, 1)); % TODO get from sensor
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
            import('AE5224.rigid_body.Body.unpack');
            import('quat.Quat');
            
            % Function
            [~, q_e, ~, ~] = unpack(x);
            b_b = Quat(q_e).inv().rotate(obj.b_e);
            z = mvnrnd(b_b, obj.cov_z).';
        end
    end
end