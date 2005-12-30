function h = plot_circle(p,r,color,np)
%function h = plot_circle(p,r[,color, np])
%
%

if nargin < 4 
  np = 50;
end

nc = size(p,1);
x  = zeros(nc,np);
y  = zeros(nc,np);
th = linspace(0,2*pi,np);

for i = 1:nc
  range    = ones(1,np)*r(i);
  [xx,yy] = pol2cart(th,range);
  xx = xx + p(i,1);
  yy = yy + p(i,2);
  
  x(i,:) = xx;
  y(i,:) = yy;
end

h = line(x',y');

if nargin > 2
  set(h,'Color',color);
end

if nargout < 1
  clear h
end


