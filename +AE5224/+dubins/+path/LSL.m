classdef LSL < AE5224.dubins.path.XSY
    %LSL Dubins LSL path
    
    methods (Access = public)
        function obj = LSL(pa, ha, pb, hb, r)
            %obj = LSL(p1, h1, p2, h2)
            %   Construct LSL path
            %   - pa = Point A [x; y]
            %   - ha = Heading A [rad]
            %   - pb = Point B [x; y]
            %   - hb = Heading B [rad]
            %   - r = Turn radius
            obj@AE5224.dubins.path.XSY(pa, ha, pb, hb, r, 'L', 'L');
        end
    end
end