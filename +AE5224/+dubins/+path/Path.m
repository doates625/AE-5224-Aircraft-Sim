classdef (Abstract) Path < handle
    %PATH Dubins path
    
    properties (Access = protected)
        p0;     % Point 0 [x; y]
        p1;     % Point 1 [x; y]
        p2;     % Point 2 [x; y]
        p3;     % Point 3 [x; y]
        h0;     % Heading at p0 [rad]
        h1;     % Heading at p1 [rad]
        h2;     % Heading at p2 [rad]
        h3;     % Heading at p3 [rad]
        c01;    % Circle center p0-p1 [x; y]
        c23;    % Circle center p2-p3 [x; y]
        t01;    % Turn direction p0-p1 ['L', 'R']
        t23;    % Turn direction p2-p3 ['L', 'R']
        r;      % Turn radius
        d01;    % Distance p0-p1
        d02;    % Distance p0-p2
        d03;    % Distance p0-p3
    end
    
    methods (Access = public)
        function obj = Path(pa, ha, pb, hb, r, ta, tb)
            %obj = PATH(pa, ha, pb, hb, r, ta, tb)
            %   Construct Dubins path
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
            
            % Copy args
            obj.p0 = pa;
            obj.h0 = ha;
            obj.p3 = pb;
            obj.h3 = hb;
            obj.r = r;
            obj.t01 = ta;
            obj.t23 = tb;
            
            % Path params
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
        
        function p = get(obj, d)
            %p = GET(obj, d) Get point along path
            %   - d = Array of absolute path distances [d1, ..., dn]
            %   - p = Array of points on path [x1, ..., xn; y1, ..., yn]
            n = length(d);
            p = nan(2, n);
            for i = 1:n
                if d(i) < 0
                    continue
                elseif d(i) <= obj.d01
                    p(:, i) = obj.get_01(d(i));
                elseif d(i) <= obj.d02
                    p(:, i) = obj.get_12(d(i));
                elseif d(i) <= obj.d03
                    p(:, i) = obj.get_23(d(i));
                end
            end
        end
        
        function d = dist(obj)
            %d = DIST(obj) Get full path distance
            d = obj.d03;
        end
        
        function plot(obj, h, n)
            %PLOT(obj, n) Plot dubins path
            %   - h = Figure handle [Figure, def = figure]
            %   - n = Points to plot [int, def = 100]
            
            % Imports
            import('AE5224.dubins.util.plot_circ');
            
            % Default args
            if nargin < 2, h = figure; end
            if nargin < 3, n = 100; end
            
            % Generate points
            d = linspace(0, obj.dist(), n);
            p = obj.get(d);
            
            % Path plot
            figure(h);
            clf, hold on, grid on
            title('Dubins Path');
            xlabel('Y')
            ylabel('X')
            plot(p(2, :), p(1, :), 'b-', 'Linewidth', 2);
            plot_circ(obj.c01, obj.r, 'k--');
            plot_circ(obj.c23, obj.r, 'k--');
            axis equal
            legend('Path', 'Turn', 'Turn');
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