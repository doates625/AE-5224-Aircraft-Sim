classdef RSL < AE5224.dubins.path.Path
    %RSL Dubins RSL path
    
    methods (Access = public)
        function obj = RSL(pa, ha, pb, hb, r)
            %obj = RSL(pa, ha, pb, hb, r)
            %   Consturct RSL path
            %   - pa = Point A [x; y]
            %   - ha = Heading A [rad]
            %   - pb = Point B [x; y]
            %   - hb = Heading B [rad]
            %   - r = Turn radius
            obj@AE5224.dubins.path.Path(pa, ha, pb, hb, r, 'R', 'L');
        end
    end
end