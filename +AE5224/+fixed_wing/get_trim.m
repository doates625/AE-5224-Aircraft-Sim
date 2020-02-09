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
q_ew = sym('q_ew'); % Attitude w [quat]
q_ex = sym('q_ex'); % Attitude x [quat]
q_ey = sym('q_ey'); % Attitude y [quat]
q_ez = sym('q_ez'); % Attitude z [quat]
w_bx = sym('w_ex'); % Body angle rate x [rad/s]
w_by = sym('w_ey'); % Body angle rate y [rad/s]
w_bz = sym('w_ez'); % Body angle rate z [rad/s]
d_e = sym('d_e');   % Elevator angle [rad]
d_a = sym('d_a');   % Aileron angle [rad]
d_r = sym('d_r');   % Rudder angle [rad]
d_p = sym('d_p');   % Prop thrust [0-1]

% Symbolic forces and moments
q_e = [q_ew; q_ex; q_ey; q_ez];
R_eb = Quat(q_e).conj().mat_rot();
w_b = [w_bx; w_by; w_bz];
x = body.pack(trim.p_e, q_e, trim.v_e, w_b);
u = [d_e; d_a; d_r; d_p];
[F_b, M_b] = body.forces(x, u);

% Solve equations
F_c = trim.get_F_c(body.m);
L_b = body.I_b * w_b;
w_e = trim.w_e;
eqs = [
    F_b == R_eb * [0; F_c; 0];  % Net force
    M_b == cross(w_b, L_b);     % Net moment
    w_b == R_eb * w_e;          % Body angle rate
    R_eb(2, 1) == 0;            % Point in +x
    norm(q_e) == 1;             % Unit quaternion
];
q_e0 = [1; 0; 0; 0];
w_b0 = w_e;
u0 = [0; 0; 0; 0.5];
sol = vpasolve(eqs, [q_e; w_b; u], [q_e0; w_b0; u0]);

% Trim angular conditions
q_e = double([sol.q_ew; sol.q_ex; sol.q_ey; sol.q_ez]);
w_b = double([sol.w_ex; sol.w_ey; sol.w_ez]);

% Trim controls
d_e = double(sol.d_e);
d_a = double(sol.d_a);
d_r = double(sol.d_r);
d_p = double(sol.d_p);
u = [d_e; d_a; d_r; d_p];

end