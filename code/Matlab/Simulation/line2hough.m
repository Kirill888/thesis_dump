function [r,a] = line2hough(l)
%  function [r,a] = line2hough(l)
%

%

dy = l(:,2) - l(:,4);
dx = l(:,1) - l(:,3);
y1 = l(:,2);
x1 = l(:,1);

a = atan(dy./dx) - pi/2; % a is in the range -pi -> 0
r = (dx.*y1 - dy.*x1)./(dx.*sin(a) - dy.*cos(a));

ii = find(r < 0);
r(ii) = -r(ii);
a(ii) = a(ii) + pi; % a is in the range 0 -> pi