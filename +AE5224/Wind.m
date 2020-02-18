classdef Wind < handle
    %WIND Gaussian body-frame wind turbulence simulator
    %   
    %   Author: Dan Oates (WPI Class of 2020)
    
    properties (SetAccess = protected)
        va_b;   % Air velocity Body [m/s]
    end
    
    properties (Access = protected)
        A_x;    % State matrix x
        A_y;    % State matrix y
        A_z;    % State matrix z
        B_x;    % Input matrix x
        B_y;    % Input matrix y
        B_z;    % Input matrix z
        C_x;    % Output matrix x
        C_y;    % Output matrix y
        C_z;    % Output matrix z
        v_x;    % State vector x
        v_y;    % State vector y
        v_z;    % State vecyot z
    end
    
    methods (Access = public)
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
            
            % Turbulence TF x
            num_x = std_x*sqrt(2*v_air/amp_x);
            den_x = [1, v_air/amp_x];
            tf_x = tf(num_x, den_x);
            [obj.A_x, obj.B_x, obj.C_x] = ssdata(c2d(tf_x, del_t, 'zoh'));
            
            % Turbulence TF y
            num_y = std_y*sqrt(3*v_air/amp_y)*[1, v_air/(sqrt(3)*amp_y)];
            den_y = [1, 2*v_air/amp_y, (v_air/amp_y)^2];
            tf_y = tf(num_y, den_y);
            [obj.A_y, obj.B_y, obj.C_y] = ssdata(c2d(tf_y, del_t, 'zoh'));
            
            % Turbulence TF z
            num_z = std_z*sqrt(3*v_air/amp_z)*[1, v_air/(sqrt(3)*amp_z)];
            den_z = [1, 2*v_air/amp_z, (v_air/amp_z)^2];
            tf_z = tf(num_z, den_z);
            [obj.A_z, obj.B_z, obj.C_z] = ssdata(c2d(tf_z, del_t, 'zoh'));
            
            % Initial conditions
            obj.va_b = zeros(3, 1);
            obj.v_x = zeros(size(obj.A_x, 1), 1);
            obj.v_y = zeros(size(obj.A_y, 1), 1);
            obj.v_z = zeros(size(obj.A_z, 1), 1);
        end
        
        function va_b = update(obj)
            %va_b = UPDATE(obj)
            %   Update wind simulation
            %   - va_b = Air velocity Body [m/s]
            obj.v_x = obj.A_x * obj.v_x + obj.B_x * randn();
            obj.v_y = obj.A_y * obj.v_y + obj.B_y * randn();
            obj.v_z = obj.A_z * obj.v_z + obj.B_z * randn();
            obj.va_b(1) = obj.C_x * obj.v_x;
            obj.va_b(2) = obj.C_y * obj.v_y;
            obj.va_b(3) = obj.C_z * obj.v_z;
            va_b = obj.va_b;
        end
    end
end