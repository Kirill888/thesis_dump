function h = plot_corner(corners, r)
% function h = plot_corner(corners, r)

%
if nargin < 2
  r = 0.1;
end


ii = find(corners(:,5) == 1 | corners(:,5) == 2);
h = plot_convex(corners(ii,:),r);

ii2 = find(corners(:,5) == 3);
h = [h; plot_jump(corners(ii2,:),r)];


function h = plot_convex(corners,r)
  a1 = corners(:,3);  a2 = corners(:,4);
  
  x0 = corners(:,1);   y0 = corners(:,2);
  x1 = r*cos(a1) + x0; y1 = r*sin(a1) + y0;
  x2 = r*cos(a2) + x0; y2 = r*sin(a2) + y0;

  lx = [x1'; x0'; x2'];
  ly = [y1'; y0'; y2'];
  
  h = line(lx,ly);
     
function h = plot_jump(cc,r)
  h = [];
     