function a = h_dif(h1, h2, s)
%a = H_DIF(h1, h2, s)
%   Angle difference from h1 to h2
%   
%   Inputs:
%   - h1 = Heading 1 [rad]
%   - h2 = Heading 2 [rad]
%   - s = Turn sign [+1, -1]
%   
%   Outputs:
%   - a = Turn angle [rad]

a = mod(s * (h2 - h1), 2*pi);

end