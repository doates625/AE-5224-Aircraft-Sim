function [p_e, v_e] = unpack_z(z)
%[p_e, v_e] = UNPACK_Z(z)
%   Get components from output vector
%   
%   Inputs:
%   - z = Output vector
%   
%   Outputs:
%   - p_e = Position Earth [m]
%   - v_e = Velocity Earth [m/s]

p_e = z(1:3);
v_e = z(4:6);

end