function c = circle(p, h, r, t)
%c = CIRCLE(p, h, r, t
%   Dubins circle center
%   
%   Inputs:
%   - p = Position [x; y]
%   - h = Heading [rad]
%   - r = Turn radius
%   - t = Direction ['L', 'R']
%   
%   Outputs:
%   - c = Center [x; y]

sgn = (t == 'L') - (t == 'R');
c = p + sgn * r * [+sin(h); -cos(h)];

end