function [x_st, u_st] = solve(model, trim)
%[x_st, u_st] = SOLVE(model, trim)
%   Solve trim conditions for fixed-wing aircraft
%   
%   Inputs:
%   - model = Fixed-wing model [AE5224.fixed_wing.Model]
%   - trim = Trim conditions [AE5224.Trim]
%   
%   Outputs:
%   - x_st = Trim states [p_e; q_e; v_e; w_b]
%   - u_st = Trim controls [d_e; d_a; d_r; d_p]

% Imports
import('AE5224.rigid_body.Model');
import('quat.Quat');

% Symbolic unknowns
q_e = sym('q_e', [4, 1]);
w_b = sym('w_b', [3, 1]);
u_st = sym('u', [4, 1]);
va_b = zeros(3, 1);

% Symbolic dynamics
R_eb = Quat(q_e).conj().mat_rot();
x_st = Model.pack_x(trim.p_e, q_e, trim.v_e, w_b);
x_dot = model.dynamics(x_st, [u_st; va_b]);
[~, ~, v_e_dot, w_b_dot] = Model.unpack_x(x_dot);

% Numerical solver
eqs = [
    v_e_dot == trim.V^2 / trim.R * [0; 1; 0];
    w_b_dot == 0;
    w_b == R_eb * trim.w_e;
    R_eb(2, 1) == 0
    norm(q_e) == 1
];
q_e0 = [1; 0; 0; 0];
w_b0 = trim.w_e;
u0 = [0; 0; 0; 0.5];
sol = vpasolve(eqs, [q_e; w_b; u_st], [q_e0; w_b0; u0]);

% Numerical evaluations
q_e = double([sol.q_e1; sol.q_e2; sol.q_e3; sol.q_e4]);
w_b = double([sol.w_b1; sol.w_b2; sol.w_b3]);
x_st = Model.pack_x(trim.p_e, q_e, trim.v_e, w_b);
u_st = double([sol.u1; sol.u2; sol.u3; sol.u4]);

end