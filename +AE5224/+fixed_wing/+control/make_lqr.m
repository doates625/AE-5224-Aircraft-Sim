function [K, Q, R] = make_lqr(A, B, dx_max, u_st, u_min, u_max, verbose)
%[K, Q, R] = MAKE_LQR(A, B, dx_max, u_st, u_min, u_max, verbose)
%   Design anti-saturation LQR
%   
%   This function designs a feedback gain K for the linearized system:
%   
%       d/dt(dx) = A*dx + B*du
%       du = -K*dx
%       dx = x - x_st
%       du = u - u_st
%       u_min <= u <= u_max
%   
%   Using LQR. The Q and R costs are initialized to identity. The input costs
%   R are increased until no inputs saturate for dx in [-dx_max, +dx_max].
%   
%   Inputs:
%   - A = State matrix
%   - B = Input matrix
%   - x_st = Trim state
%   - dx_max = Max state devs
%   - u_st = Trim controls
%   - u_min = Min controls
%   - u_max = Max controls
%   - verbose = Print flag [logical, def = true]
%   
%   Outputs:
%   - K = Feedback matrix
%   - Q = State cost matrix
%   - R = Input cost matrix

% Default args
if nargin < 7, verbose = true; end

% Initial costs
n = length(dx_max);
m = length(u_st);
Q = eye(n);
R = eye(m);

% Increase R
dx_lim = {-dx_max, +dx_max};
sat = true;
while sat
    % Set feedback gain
    K = lqr(A, B, Q, R);
    sat = false;

    % For each state
    for i = 1:n
        dx = zeros(n, 1);
        
        % For each limit direction
        for d = 1:2
            dx(i) = dx_lim{d}(i);
            u = u_st - K * dx;
            
            % For each control
            for j = 1:m
                if u(j) > u_max(j) || u(j) < u_min(j)
                    R(j, j) = R(j, j) * 1.1;
                    sat = true;
                end
            end
        end
    end
end

% Printouts
if verbose
    disp('LQR Design:')
    disp('Q = ')
    disp(Q)
    disp('R = ')
    disp(R)
    disp('K = ')
    disp(K)
end

end