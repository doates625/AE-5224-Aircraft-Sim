function ang = turn_L(h1, h2)
%ang = TURN_L(h1, h2)
%   Left turn angle from h1 to h2
%   - h1 = Heading 1 [rad]
%   - h2 = Heading 2 [rad]
%   - ang = Turn angle [0, 2pi]

ang = mod(h1 - h2, 2*pi);

end