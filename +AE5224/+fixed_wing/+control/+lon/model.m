function [A, B, x, u] = model(body, x_st, u_st)
%[A, B, x, u] = MODEL(body, x_st, u_st)
%   Linearized longitudinal (lon) flight model
%   
%   Inputs:
%   - body = Fixed-wing model [AE5224.fixed_wing.Body]
%   - x_st = Trim state vector [p_e; q_e; v_e; w_b]
%   - u_st = Trim control vector [d_e; d_a; d_r; d_p]
%   
%   Outputs:
%   - A_sub = Linearized lon state matrix
%   - B_sub = Linearized lon input matrix
%   - x = Trim lon state vector
%   - u = Trim lon control vector

% Imports
import('AE5224.fixed_wing.control.lon.x_to_xlon');
import('AE5224.fixed_wing.control.lon.u_to_ulon');
import('AE5224.fixed_wing.control.sub_model');

% Function
[A, B, x, u] = sub_model(body, x_st, u_st, @x_to_xlon, @u_to_ulon);

end