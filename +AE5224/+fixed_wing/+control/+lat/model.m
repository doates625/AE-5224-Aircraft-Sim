function [A, B, x, u] = model(body, x_st, u_st)
%[A, B, x, u] = MODEL(body, x_st, u_st)
%   Linearized lateral (lat) flight model
%   
%   Inputs:
%   - body = Fixed-wing model [AE5224.fixed_wing.Body]
%   - x_st = Trim state vector [p_e; q_e; v_e; w_b]
%   - u_st = Trim control vector [d_e; d_a; d_r; d_p]
%   
%   Outputs:
%   - A_sub = Linearized lat state matrix
%   - B_sub = Linearized lat input matrix
%   - x = Trim lat state vector
%   - u = Trim lat control vector

% Imports
import('AE5224.fixed_wing.control.lat.x_to_xlat');
import('AE5224.fixed_wing.control.lat.u_to_ulat');
import('AE5224.fixed_wing.control.sub_model');

% Function
[A, B, x, u] = sub_model(body, x_st, u_st, @x_to_xlat, @u_to_ulat);

end