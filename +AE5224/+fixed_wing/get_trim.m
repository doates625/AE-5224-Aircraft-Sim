function [q_e, w_b, u] = get_trim(body, trim)
%[q_e, w_b, u] = GET_TRIM(body, trim)
%   Get trim conditions for fixed-wing aircraft
%   
%   Inputs:
%   - body = Fixed-wing model [AE5224.fixed_wing.Body]
%   - trim = Trim conditions [AE5224.Trim]
%   
%   Outputs:
%   - q_e = Init Earth pose [quaternion]
%   - w_b = Init Body angle rate [rad/s]
%   - u = Trim controls [d_e; d_a; d_r; d_p]

% Imports
import('quat.Quat');

% Symbolic unknowns
q_e = sym('q_e', [4, 1]);
w_b = sym('w_b', [3, 1]);
u = sym('u', [4, 1]);

% Symbolic forces and moments
R_eb = Quat(q_e).conj().mat_rot();
x = body.pack(trim.p_e, q_e, trim.v_e, w_b);
[F_b, M_b] = body.forces(x, u);

% Solve equations
F_c = trim.get_F_c(body.m);
L_b = body.I_b * w_b;
eqs = [
    F_b == R_eb * [0; F_c; 0];
    M_b == cross(w_b, L_b);
    w_b == R_eb * trim.w_e;
    R_eb(2, 1) == 0;
    norm(q_e) == 1;
];
q_e0 = [1; 0; 0; 0];
w_b0 = trim.w_e;
u0 = [0; 0; 0; 0.5];
sol = vpasolve(eqs, [q_e; w_b; u], [q_e0; w_b0; u0]);

% Numerical evaluations
q_e = double(sol.q_e);
w_b = double(sol.w_b);
u = double(sol.u);

end