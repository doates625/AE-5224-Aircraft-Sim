function path = get_path(pa, ha, pb, hb, r)
%path = GET_PATH(pa, ha, pb, hb, r)
%   Get shortest dubins path from A to B
%   
%   Inputs:
%   - pa = Point A [x; y]
%   - ha = Heading A [rad]
%   - pb = Point B [x; y]
%   - hb = Heading B [rad]
%   - r = Turn radius
%   
%   Outputs:
%   - path = Path [AE5224.dubins.path.Path]

% Imports
import('AE5224.dubins.path.LSL');
import('AE5224.dubins.path.LSR');
import('AE5224.dubins.path.RSL');
import('AE5224.dubins.path.RSR');

% Find path
paths = {@LSL, @LSR, @RSL, @RSR};
best_path = [];
best_dist = inf;
for i = 1:4
    % Try to make path (TODO revert)
    try path = paths{i}(pa, ha, pb, hb, r);
    catch; continue; end
    
    % Compare to best
    dist = path.dist();
    if dist < best_dist
        best_path = path;
        best_dist = dist;
    end
end
path = best_path;

end