function ang = turn_R(h1, h2)
%ang = TURN_R(h1, h2)
%   Right turn angle from h1 to h2
%   - h1 = Heading 1 [rad]
%   - h2 = Heading 2 [rad]
%   - ang = Turn angle [0, 2pi]

ang = mod(h2 - h1, 2*pi);

end