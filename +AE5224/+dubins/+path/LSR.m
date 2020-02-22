classdef LSR < AE5224.dubins.path.XSY
    %LSR Dubins LSR path
    
    methods (Access = public)
        function obj = LSR(pa, ha, pb, hb, r)
            %obj = LSR(p1, h1, p2, h2)
            %   Construct LSR path
            %   - pa = Point A [x; y]
            %   - ha = Heading A [rad]
            %   - pb = Point B [x; y]
            %   - hb = Heading B [rad]
            %   - r = Turn radius
            obj@AE5224.dubins.path.XSY(pa, ha, pb, hb, r, 'L', 'R');
        end
    end
end