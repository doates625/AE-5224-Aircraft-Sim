function [A_sub, B_sub, x_sub, u_sub] = sub_model(...
    model, x_st, u_st, ...
    x_to_xsub, xsub_to_x_ref, ...
    u_to_usub, usub_to_u_ref)
%[A, B, x, u] = SUB_MODEL(...
%   model, x_st, u_st,...
%   x_to_xsub, xsub_to_x_ref, ...
%   u_to_usub, usub_to_u_ref)
%   Linearized flight sub-model at trim
%   
%   Inputs:
%   - model = Fixed-wing model [AE5224.fixed_wing.Body]
%   - x_st = Trim state vector [p_e; q_e; v_e; w_b]
%   - u_st = Trim control vector [d_e; d_a; d_r; d_p]
%   - x_to_xsub = Func : x -> x_sub
%   - u_to_usub = Func : u -> u_sub
%   - xsub_to_x_ref = Func : x_sub, x_ref -> x
%   - usub_to_u_ref = Func : u_sub, u_ref -> u
%   
%   Outputs:
%   - A_sub = Linearized sub-state matrix
%   - B_sub = Linearized sub-input matrix
%   - x_sub = Trim sub-state vector
%   - u_sub = Trim sub-control vector

% Trim sub-vectors
x_sub = x_to_xsub(x_st);
u_sub = u_to_usub(u_st);

% Conversion functions
x_to_xdot = @(x) model.dynamics(x, u_st);
u_to_xdot = @(u) model.dynamics(x_st, u);
xsub_to_x = @(x_sub) xsub_to_x_ref(x_sub, x_st);
usub_to_u = @(u_sub) usub_to_u_ref(u_sub, u_st);

% Intermediate Jacobians
del_xdot_x = jacnum(x_to_xdot, x_st);
del_xdot_u = jacnum(u_to_xdot, u_st);
del_xsub_x = jacnum(x_to_xsub, x_st);
del_x_xsub = jacnum(xsub_to_x, x_sub);
del_u_usub = jacnum(usub_to_u, u_sub);

% Lon Jacobians
A_sub = del_xsub_x * del_xdot_x * del_x_xsub;
B_sub = del_xsub_x * del_xdot_u * del_u_usub;

end