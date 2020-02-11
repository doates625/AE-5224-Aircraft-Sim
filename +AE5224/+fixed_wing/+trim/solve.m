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
import('AE5224.rigid_body.Model.pack_x');
import('quat.Quat');

% Symbolic unknowns
q_e = sym('q_e', [4, 1]);
w_b = sym('w_b', [3, 1]);
u_st = sym('u', [4, 1]);

% Symbolic forces and moments
R_eb = Quat(q_e).conj().mat_rot();
x_st = model.pack_x(trim.p_e, q_e, trim.v_e, w_b);
[F_b, M_b] = model.forces(x_st, u_st);

% Solve equations
F_c = trim.get_F_c(model.m);
L_b = model.I_b * w_b;
eqs = [
    F_b == R_eb * [0; F_c; 0];  % Net forces
    M_b == cross(w_b, L_b);     % Net moments
    w_b == R_eb * trim.w_e;     % Body angle rate
    R_eb(2, 1) == 0;            % Zero yaw
    norm(q_e) == 1;             % Unit quaternion
];
q_e0 = [1; 0; 0; 0];
w_b0 = trim.w_e;
u0 = [0; 0; 0; 0.5];
sol = vpasolve(eqs, [q_e; w_b; u_st], [q_e0; w_b0; u0]);

% Numerical evaluations
q_e = double([sol.q_e1; sol.q_e2; sol.q_e3; sol.q_e4]);
w_b = double([sol.w_b1; sol.w_b2; sol.w_b3]);
x_st = pack_x(trim.p_e, q_e, trim.v_e, w_b);
u_st = double([sol.u1; sol.u2; sol.u3; sol.u4]);

end