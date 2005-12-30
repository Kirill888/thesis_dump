function h = plot_coord(odo,r)
% function h = plot_coord(odo,r)
%
%

%
sa = sin(odo(:,3));
ca = cos(odo(:,3));
zz = zeros(size(odo(:,3)));
xx = odo(:,1);
yy = odo(:,2);

X = r*[-sa,zz, ca] + [xx,xx,xx];
Y = r*[ ca,zz, sa] + [yy,yy,yy];

h = plot(X',Y','r-');
