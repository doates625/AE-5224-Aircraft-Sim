function [A, B, x, u] = sub_model(body, x_st, u_st, x_to_xsub, u_to_usub)
%[A, B, x, u] = SUB_MODEL(body, x_st, u_st, x_to_xsub, u_to_usub)
%   Linearized flight sub-model at trim
%   
%   Inputs:
%   - body = Fixed-wing model [AE5224.fixed_wing.Body]
%   - x_st = Trim state vector [p_e; q_e; v_e; w_b]
%   - u_st = Trim control vector [d_e; d_a; d_r; d_p]
%   - x_to_xsub = Function from state to sub-state
%   - u_to_usub = Function from control to sub-control
%   
%   Outputs:
%   - A_sub = Linearized sub-state matrix
%   - B_sub = Linearized sub-input matrix
%   - x = Trim sub-state vector
%   - u = Trim sub-control vector

% Trim sub-vectors
x = x_to_xsub(x_st);
u = u_to_usub(u_st);

% Intermediate Jacobians
x_to_xdot = @(x) body.dynamics(x, u_st);
u_to_xdot = @(u) body.dynamics(x_st, u);
del_xdot_x = jacnum(x_to_xdot, x_st);
del_xdot_u = jacnum(u_to_xdot, u_st);
del_xsub_x = jacnum(x_to_xsub, x_st);
del_usub_u = jacnum(u_to_usub, u_st);

% Lon Jacobians
A = del_xsub_x * del_xdot_x / del_xsub_x;
B = del_xsub_x * del_xdot_u / del_usub_u;

end