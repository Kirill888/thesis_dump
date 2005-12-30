function R = ray_line(a,l, maxRange)
% function R = ray_line(a,l, maxRange)
%
%  l = [x1, y1, x2, y2]

%

sa = sin(a);
ca = cos(a);

d1 = sa.*l(:,1) - ca.*l(:,2);
d2 = sa.*l(:,3) - ca.*l(:,4);

nl = size(l,1);
R  = ones(nl,1)*inf;

ind = find(d1.*d2 <= 0);

abc = line2abc(l);

R(ind) =        - abc(ind,3)./ ...
	 (abc(ind,1).*ca + abc(ind,2).*sa );

ii = find(R > maxRange);
R(ii) = inf;
ii = find(R < 0); R(ii) = Inf;

%return
figure;
hold on

ii = find(R < maxRange);

lx = [l(:,1)'; l(:,3)'];
ly = [l(:,2)'; l(:,4)'];
hl = line(lx,ly); set(hl,'Color','b');
set(hl(ii),'Color','r');
axis equal

h = line([0 ca*maxRange],[0 sa*maxRange]); set(h,'Color','g');
x = R(ii)*ca; y = R(ii)*sa;
plot(x,y,'ro');


