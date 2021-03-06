classdef (Abstract) Path < handle
    %PATH Dubins path
    
    properties (SetAccess = protected)
        rad;    % Turn radius
    end
    
    properties (Access = protected)
        p0;     % Position 0 [x; y]
        p1;     % Position 1 [x; y]
        p2;     % Position 2 [x; y]
        p3;     % Position 3 [x; y]
        h0;     % Heading at p0 [rad]
        h1;     % Heading at p1 [rad]
        h2;     % Heading at p2 [rad]
        h3;     % Heading at p3 [rad]
        c01;    % Circle center p0-p1 [x; y]
        c23;    % Circle center p2-p3 [x; y]
        s01;    % Turn sign p0-p1 [+1, -1]
        s23;    % Turn sign p2-p3 [+1, -1]
        d01;    % Distance p0-p1
        d02;    % Distance p0-p2
        d03;    % Distance p0-p3
    end
    
    methods (Access = public)
        function obj = Path(pa, ha, pb, hb, rad, ta, tb)
            %obj = PATH(pa, ha, pb, hb, rad, ta, tb)
            %   Construct Dubins path
            %   - pa = Point A [x; y]
            %   - ha = Heading A [rad]
            %   - pb = Point B [x; y]
            %   - hb = Heading B [rad]
            %   - rad = Turn radius
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
            obj.rad = rad;
            obj.s01 = (ta == 'R') - (ta == 'L');
            obj.s23 = (tb == 'R') - (tb == 'L');
            
            % Path params
            obj.c01 = circle(obj.p0, obj.h0, obj.rad, obj.s01);
            obj.c23 = circle(obj.p3, obj.h3, obj.rad, obj.s23);
            vc = obj.c23 - obj.c01;
            [hc, dc] = cart2pol(vc(1), vc(2));
            if ta == tb
                obj.h1 = hc;
                d12 = dc;
            else
                sgn = (ta == 'R') - (ta == 'L');
                obj.h1 = hc + sgn * asin(2*obj.rad / dc);
                if ~isreal(obj.h1)
                    error('Invalid Dubins path.');
                end
                d12 = 2 * sqrt(0.25*dc^2 - obj.rad^2);
            end
            obj.h2 = obj.h1;
            d01 = rad * h_dif(obj.h0, obj.h1, obj.s01);
            d23 = rad * h_dif(obj.h2, obj.h3, obj.s23);
            obj.d01 = d01;
            obj.d02 = obj.d01 + d12;
            obj.d03 = obj.d02 + d23;
            obj.p1 = obj.get_01(obj.d01);
            obj.p2 = obj.get_12(obj.d02);
        end
        
        function d = dist(obj)
            %d = DIST(obj) Get full path distance
            d = obj.d03;
        end
        
        function [p, h, s] = get(obj, d)
            %[p, h, s] = GET(obj, d)
            %   Get point along path
            %   - d = Absolute path distance
            %   - p = Point on path [x; y]
            %   - h = Heading on path [rad]
            %   - s = Path section [1...3]
            n = length(d);
            p = nan(2, n);
            h = nan(1, n);
            s = nan(1, n);
            for i = 1:n
                if d(i) < 0
                    continue
                elseif d(i) <= obj.d01
                    [p(:, i), h(i), s(i)] = obj.get_01(d(i));
                elseif d(i) <= obj.d02
                    [p(:, i), h(i), s(i)] = obj.get_12(d(i));
                elseif d(i) <= obj.d03
                    [p(:, i), h(i), s(i)] = obj.get_23(d(i));
                end
            end
        end
        
        function sgn = sgn_a(obj)
            %sgn = SGN_A(obj) Get turn sign at A [+1, -1]
            sgn = obj.s01;
        end
        
        function sgn = sgn_b(obj)
            %sgn = SGN_B(obj) Get turn sign at B [+1, -1]
            sgn = obj.s23;
        end
        
        function err = lat_err(obj, p, s)
            %err = LAT_ERR(obj, p, s)
            %   Get lateral position error from path
            %   - p = Position [x; y]
            %   - s = Section [1...3]
            %   - err = Lateral error
            switch s
                case 1, err = obj.s01 * (obj.rad - norm(p - obj.c01));
                case 2, err = dot(p - obj.p1, [-sin(obj.h1); +cos(obj.h1)]);
                case 3, err = obj.s23 * (obj.rad - norm(p - obj.c23));
            end
        end
        
        function in = in_sect(obj, p, s)
            %in = IN_SECT(obj, p, s)
            %   Check if position is in section
            %   - p = Position [x; y]
            %   - s = Section [1...3]
            %   - in = Flag [logical]
            import('AE5224.dubins.util.h_dif');
            hp = obj.get_h(p, s);
            switch s
                case 1
                    a0p = h_dif(obj.h0, hp, obj.s01);
                    a01 = h_dif(obj.h0, obj.h1, obj.s01);
                    in = a0p < a01;
                case 2
                    v1p = p - obj.p1;
                    d1p = dot(v1p, [cos(hp); sin(hp)]);
                    in = (d1p > 0) && (d1p < obj.d02 - obj.d01);
                case 3
                    a2p = h_dif(obj.h2, hp, obj.s23);
                    a23 = h_dif(obj.h2, obj.h3, obj.s23);
                    in = a2p < a23;
            end
        end
        
        function h = get_h(obj, p, s)
            %h = GET_H(obj, p, s)
            %   Get reference heading for position
            %   - p = Position [x; y]
            %   - s = Section [1...3]
            %   - h = Heading [rad]
            switch s
                case 1
                    vcp = p - obj.c01;
                    h = atan2(vcp(2), vcp(1)) + obj.s01 * pi/2;
                case 2
                    h = obj.h1;
                case 3
                    vcp = p - obj.c23;
                    h = atan2(vcp(2), vcp(1)) + obj.s23 * pi/2;
            end
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
            plot_circ(obj.c01, obj.rad, 'k--');
            plot_circ(obj.c23, obj.rad, 'k--');
            axis equal
            legend('Path', 'Turn', 'Turn');
        end
    end
    
    methods (Access = protected)
        function [p, h, s] = get_01(obj, d)
            %[p, h] = GET_01(obj, d)
            %   Get point along path from p0 to p1
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            %   - h = Heading on path [rad]
            %   - s = Path section [1]
            import('AE5224.dubins.util.rotate');
            import('AE5224.dubins.util.h_add');
            d0x = d;
            a0x = d0x / obj.rad;
            p = rotate(obj.p0, obj.c01, a0x, obj.s01);
            h = h_add(obj.h0, a0x, obj.s01);
            s = 1;
        end

        function [p, h, s] = get_12(obj, d)
            %[p, h, s] = GET_12(obj, d)
            %   Get point along path from p1 to p2
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            %   - h = Heading on path [rad]
            %   - s = Path section [2]
            d1x = d - obj.d01;
            p = obj.p1 + d1x * [cos(obj.h1); sin(obj.h1)];
            h = obj.h1;
            s = 2;
        end

        function [p, h, s] = get_23(obj, d)
            %[p, h, s] = GET_23(obj, d)
            %   Get point along path from p2 to p3
            %   - d = Absolute distance along path
            %   - p = Point on path [x; y]
            %   - h = Heading on path [rad]
            %   - s = Path section [3]
            import('AE5224.dubins.util.rotate');
            import('AE5224.dubins.util.h_add');
            d2x = d - obj.d02;
            a2x = d2x / obj.rad;
            p = rotate(obj.p2, obj.c23, a2x, obj.s23);
            h = h_add(obj.h2, a2x, obj.s23);
            s = 3;
        end
    end
end