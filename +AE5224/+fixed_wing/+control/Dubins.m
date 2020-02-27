classdef Dubins < AE5224.control.Controller
    %DUBINS Dubins fixed-wing path tracking controller
    
    properties (Access = protected)
        path;   % Dubins path [AE5224.dubins.path.Path]
    end
    
    methods
        function obj = Dubins(pa, ha, pb, hb, R);
            %DUBINS Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

