function Df = jacnum(f, x, xd, xdd, emax)
%D = JACNUM(f, x, xd, xdd, emax)
%   Numerical Jacobian estimate
%   
%   Inputs:
%   - f = Function y = f(x) [m x 1]
%   - x = Evaluation point [n x 1]
%   - xd = Initial delta [def = 1e-3]
%   - xdd = Fractional delta of xd [def = 1e-1]
%   - emax = Max fractional error [def = 1e-6]
%   
%   Outputs:
%   - D = Jacobian [m x n]

% Default args
if nargin < 3, xd = 1e-3; end
if nargin < 4, xdd = 1e-1; end
if nargin < 5, emax = 1e-6; end

% Dimensions
y = f(x);
n = length(x);
m = length(y);
Df = zeros(m, n);

% Jacobian
for j = 1:n
    for i = 1:m
        dxj1 = xd;
        dx1 = zeros(n, 1);
        dx2 = zeros(n, 1);
        while true
            dxj2 = dxj1 * (1 - xdd);
            dx1(j) = dxj1;
            dx2(j) = dxj2;
            D1 = (f(x + dx1) - y) / dxj1;
            D2 = (f(x + dx2) - y) / dxj2;
            ecur = abs(D1(i) - D2(i)) / ((abs(D1(i)) + abs(D2(i)))/2 * xdd);
            if isnan(ecur) || abs(ecur) < emax
                Df(i, j) = D2(i);
                break
            else
                dxj1 = dxj2;
            end
        end
    end
end

end