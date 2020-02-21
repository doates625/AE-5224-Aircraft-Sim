function c = circ_L(p, h, r)
%c = CIRC_L(p, h, r)
%   Dubins left circle center
%   - p = Position [x; y]
%   - h = Heading [rad]
%   - r = Turn radius
%   - c = Center [x; y]

c = p + r * [+sin(h); -cos(h)];

end