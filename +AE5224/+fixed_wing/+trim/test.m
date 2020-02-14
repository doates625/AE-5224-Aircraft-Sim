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
import('AE5224.control.OpenLoop');
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

% TODO NOT this every time...

%{

% Lon linear model
fprintf('Lon linearization...\n');
[A_lon, B_lon, x_lon, u_lon] = ...
    AE5224.fixed_wing.control.lon.model(model, x_st, u_st);
disp_model(A_lon, B_lon, x_lon, u_lon);

% Lat linear model
fprintf('Lat linearization...\n');
[A_lat, B_lat, x_lat, u_lat] = ...
    AE5224.fixed_wing.control.lat.model(model, x_st, u_st);
disp_model(A_lat, B_lat, x_lat, u_lat);

%}

% Simulate trim
ctrl = OpenLoop(u_st);
log = simulate(model, ctrl, x_st, t_max, del_t);

function disp_model(A, B, x, u)
    %DISP_MODEL(A, B, x, u)
    %   Display linearized trim model
    %   
    %   Inputs:
    %   - A = State matrix
    %   - B = Input matrix
    %   - x = Trim state
    %   - u = Trim input
    fprintf('State Matrix:\n');
    disp(A)
    fprintf('Input Matrix:\n');
    disp(B)
    fprintf('Trim State:\n')
    disp(x)
    fprintf('Trim Control:\n')
    disp(u)
    fprintf('Eigenvalues:\n');
    disp(eig(A));
end

end