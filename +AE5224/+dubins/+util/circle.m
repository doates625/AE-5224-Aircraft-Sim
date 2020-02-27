function c = circle(p, h, r, s)
%c = CIRCLE(p, h, r, t
%   Dubins circle center
%   
%   Inputs:
%   - p = Position [x; y]
%   - h = Heading [rad]
%   - r = Turn radius
%   - s = Sign [+1, -1]
%   
%   Outputs:
%   - c = Center [x; y]

c = p - s * r * [+sin(h); -cos(h)];

end