classdef RSR < AE5224.dubins.Path
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
            
            % Imports
            import('AE5224.dubins.util.circ_R');
            import('AE5224.dubins.util.turn_R');
            
            % Construction
            obj@AE5224.dubins.Path(pa, ha, pb, hb, r);
            obj.c0 = circ_R(obj.p0, obj.h0, obj.r);
            obj.c3 = circ_R(obj.p3, obj.h3, obj.r);
            c03 = obj.c3 - obj.c0;
            [h12, d12] = cart2pol(c03(1), c03(2));
            obj.h1 = h12;
            obj.h2 = h12;
            d01 = r * turn_R(obj.h0, obj.h1);
            d23 = r * turn_R(obj.h2, obj.h3);
            obj.d01 = d01;
            obj.d02 = obj.d01 + d12;
            obj.d03 = obj.d02 + d23;
            obj.p1 = obj.get_01(obj.d01);
            obj.p2 = obj.get_12(obj.d02);
        end
    end
    
    methods (Access = protected)
        function p = get_01(obj, d)
            %p = GET_01(obj, d)
            %   Get point along path from p0 to p1
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            import('AE5224.dubins.util.rot');
            p = obj.c0 + rot(obj.p0 - obj.c0, d / obj.r);
        end
        
        function p = get_12(obj, d)
            %p = GET_12(obj, d)
            %   Get point along path from p1 to p2
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            import('AE5224.dubins.util.rot');
            p = obj.p1 + rot([d - obj.d01; 0], obj.h1);
        end
        
        function p = get_23(obj, d)
            %p = GET_23(obj, d)
            %   Get point along path from p2 to p3
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            import('AE5224.dubins.util.rot');
            p = obj.c3 + rot(obj.p3 - obj.c3, (d - obj.d03) / obj.r);
        end
    end
end