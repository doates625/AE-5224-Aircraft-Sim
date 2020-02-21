function a = angle(h1, h2, t)
%a = ANGLE(h1, h2)
%   Turn angle from h1 to h2
%   
%   Inputs:
%   - h1 = Heading 1 [rad]
%   - h2 = Heading 2 [rad]
%   - t = Direction ['L', 'R']
%   
%   Outputs:
%   - a = Turn angle [0, 2pi]

sgn = (t == 'L') - (t == 'R');
a = mod(sgn * (h1 - h2), 2*pi);

end