function z = pack_z(p_e, v_e)
%z = PACK_Z(p_e, v_e)
%   Make output vector from components
%   
%   Inputs:
%   - p_e = Position Earth [m]
%   - v_e = Velocity Earth [m/s]
%   
%   Outputs:
%   - z = Output vector

z = [p_e; v_e];

end