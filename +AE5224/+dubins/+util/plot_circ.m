function h = plot_circ(p, r, ls, n)
%h = PLOT_CIRC(p, r, n)
%   Plot circle
%   
%   Inputs:
%   - p = Origin point [x; y]
%   - r = Radius
%   - ls = Linespec [char, def = 'b-']
%   - n = Plot points [int, def = 100]
%   
%   Outputs:
%   - h = Plot handle

% Default args
if nargin < 2, ls = 'b-'; end
if nargin < 4, n = 100; end

% Plotting
th = linspace(0, 2*pi, n);
x = p(1) + r * cos(th);
y = p(2) + r * sin(th);
h = plot(y, x, ls);

end