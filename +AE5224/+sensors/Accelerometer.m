classdef Accelerometer < AE5224.sensors.Sensor
    %ACCELEROMETER Accelerometer simulator
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (Access = protected)
        v_e;    % Earth velocity [m/s]
        f_s;    % Sample rate [Hz]
        init;   % Init flag [logical]
    end
    
    methods (Access = public)
        function obj = Accelerometer(f_s, n_d)
            %obj = ACCELEROMETER(f_s, n_d)
            %   Construct accelerometer simulator
            %   
            %   Inputs:
            %   - f_s = Sample rate [Hz]
            %   - n_d = Noise density [(m/s^2)/sqrt(Hz), def = 2.45e-03]
            
            % Default args
            if nargin < 2, n_d = 2.45e-03; end
            
            % Construction
            cov_a = diag(repmat(n_d^2 * f_s, 3, 1));
            obj@AE5224.sensors.Sensor(cov_a);
            obj.v_e = zeros(3, 1);
            obj.f_s = f_s;
            obj.init = true;
        end
        
        function z = measure(obj, x)
            %z = MEASURE(obj, x)
            %   Simulate accelerometer reading
            %   
            %   Inputs:
            %   - x = State vector [p_e; q_e; v_e; w_b]
            %   
            %   Outputs:
            %   - z = Observation [a_bx; a_by; a_bz]
            
            % Imports
            import('AE5224.rigid_body.Body.unpack');
            import('AE5224.const.get_g');
            import('quat.Quat');
            
            % Unpack state
            [~, q_e, v_e_, ~] = unpack(x);
            
            % Earth acceleration
            a_e = (v_e_ - obj.v_e) * obj.f_s;
            if obj.init
                a_e = zeros(3, 1);
                obj.init = false;
            end
            obj.v_e = v_e_;
            
            % Transform to body
            a_g = [0; 0; get_g()];
            a_b = Quat(q_e).inv().rotate(a_e - a_g);
            
            % Add noise
            z = mvnrnd(a_b, obj.cov_z).';
        end
    end
end