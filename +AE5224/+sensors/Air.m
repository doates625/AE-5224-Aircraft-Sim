classdef Air < AE5224.sensors.Sensor
    %AIR 3-Axis airspeed sensor simulator
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    methods (Access = public)
        function obj = Air(cov_v)
            %obj = Air(cov_v)
            %   Construct 3-axis airspeed sensor
            %   
            %   Inputs:
            %   - cov_v = Axis velocity covariance
            
            % Default args
            if nargin < 1
                cov_v = 0.02^2;
            end
            
            % Constructor
            cov_z = cov_v * eye(3);
            obj@AE5224.sensors.Sensor(cov_z);
        end
        
        function z = measure(obj, x, va_b)
            %z = MEASURE(obj, x)
            %   Simulate airspeed reading
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; vb_e; w_b]
            %   - va_b = Air velocity Body [m/s]
            %   
            %   Outputs:
            %   - z = Observation [v_bx; v_by; v_bz]
            
            % Imports
            import('AE5224.rigid_body.Model.unpack_x');
            import('quat.Quat');
            
            % Measurement sim
            [~, q_e, vb_e, ~] = unpack_x(x);
            vb_b = Quat(q_e).inv().rotate(vb_e);
            vr_b = vb_b - va_b;
            z = mvnrnd(vr_b, obj.cov_z).';
        end
    end
end