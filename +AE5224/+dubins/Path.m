classdef (Abstract) Path < handle
    %PATH Dubins path
    
    properties (Access = protected)
        p0;     % Point 0 [x; y]
        p1;     % Point 1 [x; y]
        p2;     % Point 2 [x; y]
        p3;     % Point 3 [x; y]
        h0;     % Heading 0 [rad]
        h1;     % Heading 1 [rad]
        h2;     % Heading 2 [rad]
        h3;     % Heading 3 [rad]
        c0;     % Circle 0 [x; y]
        c3;     % Circle 3 [x; y]
        r;      % Turn radius
        d01;    % Dist 0-1
        d02;    % Dist 0-2
        d03;    % Dist 0-3
    end
    
    methods (Access = public)
        function obj = Path(pa, ha, pb, hb, r)
            %obj = PATH(p1, h1, p2, h2)
            %   Construct Dubins path
            %   - pa = Point A [x; y]
            %   - ha = Heading A [rad]
            %   - pb = Point B [x; y]
            %   - hb = Heading B [rad]
            %   - r = Turn radius
            obj.p0 = pa;
            obj.h0 = ha;
            obj.p3 = pb;
            obj.h3 = hb;
            obj.r = r;
        end
        
        function p = get(obj, d)
            %p = GET(obj, d) Get point along path
            %   - d = Array of absolute path distances [d1, ..., dn]
            %   - p = Array of points on path [x1, ..., xn; y1, ..., yn]
            n = length(d);
            p = nan(2, n);
            for i = 1:n
                if d(i) < obj.d01
                    p(:, i) = obj.get_01(d(i));
                elseif d(i) < obj.d02
                    p(:, i) = obj.get_12(d(i));
                elseif d(i) < obj.d03
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
            plot(p(2, :), p(1, :), 'b-', 'Linewidth', 2)
            plot_circ(obj.c0, obj.r, 'k--');
            plot_circ(obj.c3, obj.r, 'k--');
            axis equal
        end
    end
    
    methods (Access = protected, Abstract)
        p = get_01(obj, d)
        %p = GET_01(obj, d)
        %   Get point along path from p0 to p1
        %   - d = Absolute distance along path
        %   - p = Point on path [x; y]

        p = get_12(obj, d)
        %p = GET_12(obj, d)
        %   Get point along path from p1 to p2
        %   - d = Absolute distance along path
        %   - p = Point on path [x; y]
        
        p = get_23(obj, d)
        %p = GET_23(obj, d)
        %   Get point along path from p2 to p3
        %   - d = Absolute distance along path
        %   - p = Point on path [x; y]
    end
end