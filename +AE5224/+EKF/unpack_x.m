function [q_e, p_e, v_e, w_e, b_e] = unpack_x(x)
%[q_e, p_e, v_e, w_e, b_e] = UNPACK_X(x)
%   Get components from state vector
%   
%   Inputs:
%   - x = State vector
%   
%   Outputs:
%   - q_e = Attitude Earth [quat]
%   - p_e = Position Earth [m]
%   - v_e = Velocity Earth [m/s]
%   - w_e = Air velocity Earth [m/s]
%   - b_e = Magnetic field Earth [uT]

q_e = x(01:04);
p_e = x(05:07);
v_e = x(08:10);
w_e = x(11:13);
b_e = x(14:16);

end