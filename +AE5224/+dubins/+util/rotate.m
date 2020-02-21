function pr = rotate(p, a, t)
%pr = ROTATE(p, a) Rotate point by angle
%   
%   Inputs:
%   - p = Point [x; y]
%   - a = Angle [rad]
%   - t = Direction ['L', 'R', def = 'R']
%   
%   Outputs:
%   - pr = Rotated point [xr; yr]

% Direction
if nargin < 3, t = 'R'; end
sgn = (t == 'R') - (t == 'L');
a = sgn * a;

% Rotation
c_a = cos(a);
s_a = sin(a);
pr = [+c_a, -s_a; +s_a, +c_a] * p;

end