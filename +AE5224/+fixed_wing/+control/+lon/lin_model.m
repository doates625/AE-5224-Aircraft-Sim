function [A_lon, B_lon, x_lon, u_lon] = lin_model(model, x_st, u_st, verbose)
%[A_lon, B_lon, x_lon, u_lon] = LIN_MODEL(model, x_st, u_st, verbose)
%   Linearized longitudinal (lon) flight model
%   
%   Inputs:
%   - mode = Fixed-wing model [AE5224.fixed_wing.Body]
%   - x_st = Trim state vector [p_e; q_e; v_e; w_b]
%   - u_st = Trim control vector [d_e; d_a; d_r; d_p]
%   - verbose = Print flag [logical, def = true]
%   
%   Outputs:
%   - A_lon = Linearized lon state matrix
%   - B_lon = Linearized lon input matrix
%   - x_lon = Trim lon state vector
%   - u_lon = Trim lon control vector
%   
%   Lon state x_lon:
%   - v_bx = Velocity Body x [m/s]
%   - v_bz = Velocity Body z [m/s]
%   - w_by = Pitch rate [rad/s]
%   - th_y = Pitch angle [rad]
%   - h = Altitude [m]
%   
%   Lon input u_lon:
%   - d_e = Elevator angle [rad]
%   - d_p = Prop throttle [0-1]
%   
%   Author: Dan Oates (WPI Class of 2020)

% Imports
import('AE5224.fixed_wing.control.lon.x_to_xlon');
import('AE5224.fixed_wing.control.lon.xlon_to_x');
import('AE5224.fixed_wing.control.lon.u_to_ulon');
import('AE5224.fixed_wing.control.lon.ulon_to_u');
import('AE5224.fixed_wing.control.sub_model');

% Default args
if nargin < 4, verbose = true; end

% Numerical linearization
[A_lon, B_lon, x_lon, u_lon] = sub_model(...
    model, x_st, u_st, ...
    @x_to_xlon, @xlon_to_x, ...
    @u_to_ulon, @ulon_to_u);

% Correct known elements
A_lon(4, 1:2) = 0;
A_lon(4, 3) = 1;
A_lon(5, 3) = 0;
A_lon(3:4, 4) = 0;
A_lon(:, 5) = 0;
B_lon(4:5, 1) = 0;
B_lon(2:5, 2) = 0;

% Printouts
if verbose
    % Title
    fprintf('Lon Linearization:\n\n');
    
    % States
    fprintf('Trim States:\n');
    fprintf('- Velocity body x: %.2f [m/s]\n', x_lon(1));
    fprintf('- Velocity body z: %.2f [m/s]\n', x_lon(2));
    fprintf('- Pitch rate: %.2f [deg/s]\n', rad2deg(x_lon(3)))
    fprintf('- Pitch angle: %.2f [deg]\n\n', rad2deg(x_lon(4)))
    
    % Controls
    fprintf('Trim Controls:\n');
    fprintf('- Elevator: %.2f [deg]\n', rad2deg(u_lon(1)))
    fprintf('- Propeller: %.1f%%\n\n', 100 * u_lon(2))
    
    % Matrices
    fprintf('State Matrices:\n');
    fprintf('A_lon = \n');
    disp(A_lon);
    fprintf('B_lon = \n');
    disp(B_lon);
    
    % Eigenvalues
    fprintf('Eigenvalues:\n');
    disp(eig(A_lon))
end

end