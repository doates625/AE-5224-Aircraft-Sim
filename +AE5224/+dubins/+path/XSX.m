classdef (Abstract) XSX < AE5224.dubins.path.Path
    %XSX Superclass for LSL and RSR Dubins paths
    
    methods (Access = public)
        function obj = XSX(pa, ha, pb, hb, r, t)
            %obj = XSX(pa, ha, pb, hb, r, ta, tb)
            %   Construct XSX path
            %   - pa = Point A [x; y]
            %   - ha = Heading A [rad]
            %   - pb = Point B [x; y]
            %   - hb = Heading B [rad]
            %   - r = Turn radius
            %   - t = Turn direction ['L', 'R']
            
            % Imports
            import('AE5224.dubins.util.circle');
            import('AE5224.dubins.util.angle');
            
            obj@AE5224.dubins.path.Path(pa, ha, pb, hb, r, t, t);
            obj.c01 = circle(obj.p0, obj.h0, obj.r, obj.t01);
            obj.c23 = circle(obj.p3, obj.h3, obj.r, obj.t23);
            dc = obj.c23 - obj.c01;
            [h12, d12] = cart2pol(dc(1), dc(2));
            obj.h1 = h12;
            obj.h2 = h12;
            d01 = r * angle(obj.h0, obj.h1, obj.t01);
            d23 = r * angle(obj.h2, obj.h3, obj.t23);
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
            import('AE5224.dubins.util.rotate');
            p = obj.c01 + rotate(obj.p0 - obj.c01, d / obj.r, obj.t01);
        end
        
        function p = get_12(obj, d)
            %p = GET_12(obj, d)
            %   Get point along path from p1 to p2
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            import('AE5224.dubins.util.rotate');
            p = obj.p1 + rotate([d - obj.d01; 0], obj.h1);
        end
        
        function p = get_23(obj, d)
            %p = GET_23(obj, d)
            %   Get point along path from p2 to p3
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            import('AE5224.dubins.util.rotate');
            p = obj.c23 + rotate(...
                obj.p3 - obj.c23, (d - obj.d03) / obj.r, obj.t23);
        end
    end
end