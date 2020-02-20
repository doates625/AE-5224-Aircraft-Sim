function log = test(trim, t_max, del_t)
%log = TEST(trim, t_max, del_t)
%   Simulate and analyze trim condition
%   
%   Inputs:
%   - trim = Trim conditions [AE5224.trim.Trim]
%   - t_max = Simulation duration [s, def = 10.0]
%   - del_t = Timulation timestep [s, def = 0.01]
%   
%   Outputs:
%   - log = Simulation log [AE5224.rigid_body.Log]
clc

% Imports
import('AE5224.quad_rotor.Model');
import('AE5224.quad_rotor.trim.solve');
import('AE5224.quad_rotor.Log');
import('AE5224.control.OpenLoop');
import('AE5224.rigid_body.Model.unpack_x');
import('AE5224.simulate');
import('quat.Quat');

% Default args
if nargin < 1, t_max = 10.0; end
if nargin < 2, del_t = 0.01; end

% Initial printout
fprintf('Quadrotor Trim Test\n\n');

% Trim solver
fprintf('Solving for trim state...\n\n');
model = Model();
[x_st, u_st] = solve(model, trim);

% Trim states
[~, q_e, v_e, w_b] = unpack_x(x_st);
q_e = Quat(q_e);
[~, ty, tx] = q_e.euler();
v_b = q_e.inv().rotate(v_e);
fprintf('Trim States:\n');
fprintf('- Pitch angle: %.2f [rad]\n', ty);
fprintf('- Roll angle: %.2f [rad]\n', tx);
fprintf('- Body velocity: [%+.2f, %+.2f, %+.2f] [rad]\n', v_b)
fprintf('- Body angular: [%+.2f, %+.2f, %+.2f] [rad/s]\n\n', w_b);

% Trim controls
fprintf('Trim Controls:\n');
fprintf('- Prop 1: %.0f [rpm]\n', u_st(1));
fprintf('- Prop 2: %.0f [rpm]\n', u_st(2));
fprintf('- Prop 3: %.0f [rpm]\n', u_st(3));
fprintf('- Prop 4: %.0f [rpm]\n\n', u_st(4));

% Simulate trim
ctrl = OpenLoop(u_st, model.u_min, model.u_max);
sim_wind = false;
ekf_fb = false;
log = simulate(model, ctrl, sim_wind, ekf_fb, x_st, t_max, del_t, @Log);

end