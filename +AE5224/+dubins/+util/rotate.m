function pr = rotate(p, c, a, s)
%pr = ROTATE(p, c, a, s) Rotate point about center
%   
%   Inputs:
%   - p = Point [x; y]
%   - c = Center [x; y]
%   - a = Angle [rad]
%   - s = Turn sign [+1, -1]
%   
%   Outputs:
%   - pr = Rotated point [xr; yr]

% Direction
if nargin < 3, s = +1; end
a = s * a;

% Rotation
c_a = cos(a);
s_a = sin(a);
pr = c + [+c_a, -s_a; +s_a, +c_a] * (p - c);

end