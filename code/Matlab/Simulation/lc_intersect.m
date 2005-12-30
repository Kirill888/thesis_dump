function p = lc_intersect(l,c)
% function p = lc_intersect(l,c)
%
% Finds intersection of the line with the circle.
%
%  Line is defined by two points l(1,1:2) - point 1 (x,y)
%                                l(2,1:2) - point 2 (x,y)
%  Circle is defined by a triplet
%                                c(1:2)   - center (x,y)
%                                c(3)     - radius
%  Returns:
%   p = empty    -- No intersection.
%   p = [x1,y1;
%        x2,y2]  -- Intersection at two points.
%   p = [x,y]    -- Line touches the circle at this point.
%

%

p1 = l(1,1:2);
p2 = l(2,1:2);
p3 = c(1:2);
r  = c(3);

dp = p2 - p1;
dc = p1 - p3;

a = dp*dp';
b = 2*dp*dc';
c = p3*p3' + p1*p1' - 2*p1*p3' - r*r;

D = b*b - 4*a*c;

if D < 0
  p = zeros(0,2);
elseif D == 0
  u = -b/(2*a);
  p = p1 + u*dp;
else
  u1 = (-b + sqrt(D))/(2*a);
  u2 = (-b - sqrt(D))/(2*a);
  
  p(1,1:2) = p1 + u1*dp;
  p(2,1:2) = p1 + u2*dp;
end

return
% GUI output
figure
line(l(:,1),l(:,2));
th = linspace(0,2*pi,100);
range  = ones(1,100)*r;
[x,y] = pol2cart(th,range);
x = x + p3(1);
y = y + p3(2);
hold on
line(x,y);

plot(p(:,1),p(:,2),'bs');
axis equal

