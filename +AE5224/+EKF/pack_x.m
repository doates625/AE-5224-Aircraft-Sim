function x = pack_x(q_e, p_e, v_e, w_e, b_e)
%x = PACK_X(q_e, p_e, v_e, w_e, b_e)
%   Make state vector from components
%   
%   Inputs:
%   - q_e = Attitude Earth [quat]
%   - p_e = Position Earth [m]
%   - v_e = Velocity Earth [m/s]
%   - w_e = Air velocity Earth [m/s]
%   - b_e = Magnetic field Earth [uT]
%   
%   Outputs:
%   - x = State vector

x = [q_e; p_e; v_e; w_e; b_e];

end