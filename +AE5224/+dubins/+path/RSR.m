classdef RSR < AE5224.dubins.path.Path
    %RSR Dubins RSR path
    
    methods (Access = public)
        function obj = RSR(pa, ha, pb, hb, r)
            %obj = RSR(p1, h1, p2, h2)
            %   Construct RSR path
            %   - pa = Point A [x; y]
            %   - ha = Heading A [rad]
            %   - pb = Point B [x; y]
            %   - hb = Heading B [rad]
            %   - r = Turn radius
            obj@AE5224.dubins.path.Path(pa, ha, pb, hb, r, 'R', 'R');
        end
    end
end