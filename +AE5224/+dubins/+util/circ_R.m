function c = circ_R(p, h, r)
%c = CIRC_R(p, h, r)
%   Dubins right circle center
%   - p = Position [x; y]
%   - h = Heading [rad]
%   - r = Turn radius
%   - c = Center [x; y]

c = p - r * [+sin(h); -cos(h)];

end