function h = plot_corner_map(mm,R)
% function plot_corner_map(mm)
%
%


%

if nargin < 2
  R = 0.3;
end


type = mm(:,7);
A1   = mm(:,8);
A2   = mm(:,9);
x0   = mm(:,1);
y0   = mm(:,2);

ii1 = find(type > 1);  %These only have one arm jump cvx hidden
ii2 = find(type <= 1); %Convex and concave

x1   = R*cos(A1(ii1)) + x0(ii1);
y1   = R*sin(A1(ii1)) + y0(ii1);

x2_1 = R*cos(A1(ii2)) + x0(ii2);
x2_2 = R*cos(A2(ii2)) + x0(ii2);

y2_1 = R*sin(A1(ii2)) + y0(ii2);
y2_2 = R*sin(A2(ii2)) + y0(ii2);

h = plot([x0(ii1),x1]',[y0(ii1),y1]','r-', ...
	 [x2_1, x0(ii2), x2_2]', [y2_1,y0(ii2),y2_2]', 'r-');
       %,x0,y0,'b.'); %,x2_1,y2_1,'g^');

if nargout == 0
  clear h
end

