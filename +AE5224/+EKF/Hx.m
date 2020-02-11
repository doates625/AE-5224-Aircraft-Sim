function Hx_ = Hx(~)
%Hx_ = HX(x)
%   Get output Jacobian
%   - x = State vector
%   - Hx = Output Jacobian

Hx_ = [zeros(6, 4), eye(6, 6), zeros(6, 6)];

end