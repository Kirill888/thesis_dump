function h = plot_grid(data, dimensions)
% function h = plot_grid(data, dimensions)
%
%

if nargin == 1
  dimensions = data.dim;
  data = data.data;
end

xx = linspace(dimensions(1), dimensions(3), size(data,2));
yy = linspace(dimensions(2), dimensions(4), size(data,1));

h = pcolor(xx,yy,data);



