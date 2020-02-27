function h2 = h_add(h1, a, s)
%h2 = H_ADD(h1, a, t)
%   Add angle to heading
%   
%   Inputs:
%   - h1 = Heading 1 [rad]
%   - a = Turn angle [rad]
%   - s = Turn sign [+1, -1]
%   
%   Outputs:
%   - h2 = Heading 2 [rad]

h2 = mod(h1 + s * a, 2*pi);

end