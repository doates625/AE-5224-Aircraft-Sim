classdef Wind < handle
    %WIND Gaussian body-frame wind turbulence simulator
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        v_air;  % Trim airspeed [m/s]
        del_t;  % Sim time delta [s]
        std_x;  % Turbulence x [m/s]
        std_y;  % Turbulence y [m/s]
        std_z;  % Turbulence z [m/s]
        amp_x;  % Spatial amp x [m]
        amp_y;  % Spatial amp y [m]
        amp_z;  % Spatial amp z [m]
    end
    
    methods
        function obj = Wind(v_air, del_t, std_x, std_y, std_z, amp_x, amp_y, amp_z)
            %obj = WIND(v_air, del_t, std_x, std_y, std_z, amp_x, amp_y, amp_z)
            %   Create wind turbulence simulator
            %   
            %   Inputs:
            %   - v_air = Trim airspeed [m/s]
            %   - del_t = Sim time delta [s]
            %   - std_x = Turbulence x [m/s]
            %   - std_y = Turbulence y [m/s]
            %   - std_z = Turbulence z [m/s]
            %   - amp_x = Spatial amp x [m]
            %   - amp_y = Spatial amp y [m]
            %   - amp_z = Spatial amp z [m]
            %   
            %   Default params from textbook
            
            % Default args
            if nargin < 3, std_x = 2.12; end
            if nargin < 4, std_y = 2.12; end
            if nargin < 5, std_z = 1.40; end
            if nargin < 6, amp_x = 200; end
            if nargin < 7, amp_y = 200; end
            if nargin < 8, amp_z = 50; end
            
            % Construction
            obj.v_air = v_air;
            obj.del_t = del_t;
            obj.std_x = std_x;
            obj.std_y = std_y;
            obj.std_z = std_z;
            obj.amp_x = amp_x;
            obj.amp_y = amp_y;
            obj.amp_z = amp_z;
        end
        
        function w_b = update(obj)
            %w_b = UPDATE(obj)
            %   TODO find better name (w_b is already angular rate)
            %   TODO simulate transfer functions in SS form (tf2ss, u = gauss)
            w_b = zeros(3, 1);
        end
    end
end