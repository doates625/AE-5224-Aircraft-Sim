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
        s01;    % Turn sign p0-p1 [-1, +1]
        s23;    % Turn sign p2-p3 [-1, +1]
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
            import('AE5224.dubins.util.h_dif');
            
            % Copy args
            obj.p0 = pa;
            obj.h0 = ha;
            obj.p3 = pb;
            obj.h3 = hb;
            obj.r = r;
            obj.s01 = (ta == 'R') - (ta == 'L');
            obj.s23 = (tb == 'R') - (tb == 'L');
            
            % Path params
            obj.c01 = circle(obj.p0, obj.h0, obj.r, obj.s01);
            obj.c23 = circle(obj.p3, obj.h3, obj.r, obj.s23);
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
            d01 = r * h_dif(obj.h0, obj.h1, obj.s01);
            d23 = r * h_dif(obj.h2, obj.h3, obj.s23);
            obj.d01 = d01;
            obj.d02 = obj.d01 + d12;
            obj.d03 = obj.d02 + d23;
            obj.p1 = obj.get_01(obj.d01);
            obj.p2 = obj.get_12(obj.d02);
        end
        
        function [p, h] = get(obj, d)
            %p = GET(obj, d) Get point along path
            %   - d = Absolute path distances [d1, ..., dn]
            %   - p = Points on path [x1, ..., xn; y1, ..., yn]
            %   - h = Headings on path [h1, ... hn] [rad]
            n = length(d);
            p = nan(2, n);
            h = nan(1, n);
            for i = 1:n
                if d(i) < 0
                    continue
                elseif d(i) <= obj.d01
                    [p(:, i), h(i)] = obj.get_01(d(i));
                elseif d(i) <= obj.d02
                    [p(:, i), h(i)] = obj.get_12(d(i));
                elseif d(i) <= obj.d03
                    [p(:, i), h(i)] = obj.get_23(d(i));
                end
            end
        end
        
        function d = dist(obj)
            %d = DIST(obj) Get full path distance
            d = obj.d03;
        end
        
        function plot(obj, fig, n)
            %PLOT(obj, fig, n) Plot dubins path
            %   - fig = Figure handle [Figure, def = figure]
            %   - n = Points to plot [int, def = 100]
            
            % Imports
            import('AE5224.dubins.util.plot_circ');
            
            % Default args
            if nargin < 2, fig = figure; end
            if nargin < 3, n = 100; end
            
            % Generate points
            d = linspace(0, obj.dist(), n);
            [p, h] = obj.get(d);
            
            % Path plot
            figure(fig);
            clf, hold on, grid on
            title('Dubins Path');
            xlabel('Y')
            ylabel('X')
            plot3(p(2, :), p(1, :), h, 'b-', 'Linewidth', 2);
            plot_circ(obj.c01, obj.r, 'k--');
            plot_circ(obj.c23, obj.r, 'k--');
            axis equal
            legend('Path', 'Turn', 'Turn');
        end
    end
    
    methods (Access = protected)
        function [p, h] = get_01(obj, d)
            %[p, h] = GET_01(obj, d)
            %   Get point along path from p0 to p1
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            %   - h = Heading on path [rad]
            import('AE5224.dubins.util.rotate');
            import('AE5224.dubins.util.h_add');
            d0x = d;
            a0x = d0x / obj.r;
            p = rotate(obj.p0, obj.c01, a0x, obj.s01);
            h = h_add(obj.h0, a0x, obj.s01);
        end

        function [p, h] = get_12(obj, d)
            %[p, h] = GET_12(obj, d)
            %   Get point along path from p1 to p2
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            %   - h = Heading on path [rad]
            d1x = d - obj.d01;
            p = obj.p1 + d1x * [cos(obj.h1); sin(obj.h1)];
            h = obj.h1;
        end

        function [p, h] = get_23(obj, d)
            %[p, h] = GET_23(obj, d)
            %   Get point along path from p2 to p3
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            %   - h = Heading on path [rad]
            import('AE5224.dubins.util.rotate');
            import('AE5224.dubins.util.h_add');
            d2x = d - obj.d02;
            a2x = d2x / obj.r;
            p = rotate(obj.p2, obj.c23, a2x, obj.s23);
            h = h_add(obj.h2, a2x, obj.s23);
        end
    end
end