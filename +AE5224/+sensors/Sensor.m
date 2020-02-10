classdef (Abstract) Sensor < handle
    %SENSOR Class for simulated zero-mean Gaussian sensor readings
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        cov_z;  % Measurement cov matrix
    end
    
    methods (Access = public)
        function obj = Sensor(cov_z)
            %obj = SENSOR(cov_z)
            %   Construct sensor simulator
            %   
            %   Inputs:
            %   - cov_z = Measurement cov matrix
            obj.cov_z = cov_z;
        end
    end
    
    methods (Access = public, Abstract)
        z = measure(obj, x)
        %z = MEASURE(obj, x)
        %   Simulate sensor reading
        %   
        %   Inputs:
        %   - x = State vector [p_e; q_e; v_e; w_b]
        %   
        %   Outputs:
        %   - z = Observation vector
    end
end

