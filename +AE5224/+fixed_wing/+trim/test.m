function log = test(trim, open_loop, sim_wind, ekf_fb, t_max, del_t)
%log = TEST(trim, file, open_loop, sim_wind, t_max, del_t)
%   Simulate and analyze trim condition
%   
%   Inputs:
%   - trim = Trim conditions [AE5224.trim.Trim]
%   - open_loop = Open loop control flag [logical]
%   - sim_wind = Wind simulation flag [logical]
%   - ekf_fb = EKF feedback flag [logical]
%   - t_max = Simulation duration [s]
%   - del_t = Timulation timestep [s]
%   
%   Outputs:
%   - log = Simulation log [AE5224.rigid_body.Log]
clc

% Imports
import('AE5224.fixed_wing.Model');
import('AE5224.fixed_wing.trim.solve');
import('AE5224.fixed_wing.control.Trim');
import('AE5224.fixed_wing.Log');
import('AE5224.control.OpenLoop');
import('AE5224.sim');

% Initial printout
fprintf('Fixedwing Trim Test\n\n');

% Trim solver
fprintf('Solving for trim state...\n');
model = Model();
[x_st, u_st] = solve(model, trim);

% Control design
if open_loop
    ctrl = OpenLoop(model, u_st);
else
    fprintf('Designing linear controller...\n');
    ctrl = Trim(model, x_st, u_st);
    disp(ctrl);
end

% Simulate trim
log = sim(model, ctrl, sim_wind, ekf_fb, x_st, t_max, del_t, @Log);

end