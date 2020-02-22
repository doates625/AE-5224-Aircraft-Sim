classdef XSY < AE5224.dubins.path.Path
    %XSY Superclass for LSL, LSR, RSL, and RSR Dubins paths
    
    methods (Access = public)
        function obj  = XSY(pa, ha, pb, hb, r, ta, tb)
            %obj = XSX(pa, ha, pb, hb, r, ta, tb)
            %   Construct XSX path
            %   - pa = Point A [x; y]
            %   - ha = Heading A [rad]
            %   - pb = Point B [x; y]
            %   - hb = Heading B [rad]
            %   - r = Turn radius
            %   - ta = Turn A ['L', 'R']
            %   - tb = Turn B ['L', 'R']
            
            % Imports
            import('AE5224.dubins.util.circle');
            import('AE5224.dubins.util.angle');
            
            % Construction
            obj@AE5224.dubins.path.Path(pa, ha, pb, hb, r, ta, tb);
            obj.c01 = circle(obj.p0, obj.h0, obj.r, obj.t01);
            obj.c23 = circle(obj.p3, obj.h3, obj.r, obj.t23);
            vc = obj.c23 - obj.c01;
            [hc, dc] = cart2pol(vc(1), vc(2));
            if ta == tb
                obj.h1 = hc;
                d12 = dc;
            else
                sgn = (ta == 'R') - (ta == 'L');
                obj.h1 = hc + sgn * asin(2*obj.r / dc);
                if ~isreal(obj.h1)
                    error('Invalid Dubins path.');
                end
                d12 = 2 * sqrt(0.25*dc^2 - obj.r^2);
            end
            obj.h2 = obj.h1;
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