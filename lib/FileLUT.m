classdef FileLUT < handle
    %FILELUT Class for file-based lookup tables
    
    properties
        path;   % Dir path [char]
    end
    
    methods (Access = public)
        function obj = FileLUT(path)
            %obj = FILELUT(path)
            %   Create file-based LUT
            %   - path = Dir path [char]
            obj.path = path;
        end
        
        function set(obj, x, y)
            %SET(obj, x, y) Set value of x to y
            [fs, n] = obj.files();
            for i = 1:n
                f = fs{i};
                d = load(f);
                if d.x == x
                    save(f, 'x', 'y');
                    return
                end
            end
            f = [obj.path, sprintf('/lut_%03u.mat', n+1)];
            save(f, 'x', 'y');
        end
        
        function y = get(obj, x)
            %y = Get(obj, x) Get value of x
            [fs, n] = obj.files();
            for i = 1:n
                d = load(fs{i});
                if d.x == x
                    y = d.y;
                    return
                end
            end
            error('No file for x');
        end
    end
    
    methods (Access = protected)
        function [fs, n] = files(obj)
            d = dir([obj.path '/*.mat']);
            n = length(d);
            fs = cell(n, 1);
            for i = 1:n
                fs{i} = [obj.path '/' d(i).name];
            end
        end
    end
end