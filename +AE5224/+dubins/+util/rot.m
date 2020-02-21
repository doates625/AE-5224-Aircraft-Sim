function pr = rot(p, a)
%pr = ROT(p, a) Rotate point by angle
%   - p = Point [x; y]
%   - a = Angle [rad]
%   - pr = Rotated point [xr; yr]

c_a = cos(a);
s_a = sin(a);
pr = [+c_a, -s_a; +s_a, +c_a] * p;

end