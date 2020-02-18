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
import('AE5224.fixed_wing.Model');
import('AE5224.fixed_wing.trim.solve');
import('AE5224.fixed_wing.control.Controller');
import('AE5224.fixed_wing.Log');
import('AE5224.simulate');

% Default args
if nargin < 1, t_max = 10.0; end
if nargin < 2, del_t = 0.01; end

% Initial printout
fprintf('Fixedwing Trim Test\n\n');

% Trim solver
fprintf('Solving for trim state...\n');
model = Model();
[x_st, u_st] = solve(model, trim);

% Control design
fprintf('Designing linear controller...\n');
ctrl = Controller(model, x_st, u_st);

% Simulate trim
log_cls = @Log;
log = simulate(model, ctrl, log_cls, x_st, t_max, del_t);

end