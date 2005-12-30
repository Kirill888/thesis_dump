function e = compute_edge(p1, p2, do_plot)
% function e = compute_edge(p1, p2)
%
%

%

  dx  = p2(:,1) -  p1(:,1);
  dy  = p2(:,2) -  p1(:,2);
  sxx = p2(:,3) +  p1(:,3);
  sxy = p2(:,4) +  p1(:,4);
  syy = p2(:,6) +  p1(:,6);
  
  dx2 = dx.*dx; dy2 = dy.*dy; dxdy = dx.*dy;
  
  r2  = dx2 + dy2;
  r   = sqrt(r2);
  a   = atan2(dy,dx);
  
  s_r = (dx2.*sxx + dy2.*syy + 2*dxdy.*sxy)./r2;
  s_a = (dy2.*sxx + dx2.*syy - 2*dxdy.*sxy)./(r2.*r2);
  
  e = [ r , s_r ,       a , s_a];
  
  %Test Plot
  if nargin < 3
    return
  end
  
  if ~do_plot
    return
  end
  
  figure; hold on
  mplot(p1,'r.','r-'); mplot(p2,'g.','g-');
  
  np = size(p1,1);
  
  xx = [p1(:,1), p2(:,1)];
  yy = [p1(:,2), p2(:,2)];
  line(xx',yy','LineStyle',':');
  
  % plot angular uncertainty
  xo = 0.5*(xx(:,1) + xx(:,2));
  yo = 0.5*(yy(:,1) + yy(:,2));
  plot(xo,yo,'ro');
  
  da = sqrt(s_a)*3;
  dx = 0.5.*r.*cos(a + da);
  dy = 0.5.*r.*sin(a + da);
  
  xx = [xo - dx, xo + dx];
  yy = [yo - dy, yo + dy];
  line(xx',yy');

  da = sqrt(s_a)*3;
  dx = 0.5.*r.*cos(a - da);
  dy = 0.5.*r.*sin(a - da);
  
  xx = [xo - dx, xo + dx];
  yy = [yo - dy, yo + dy];
  line(xx',yy');

  txt = num2str([1:np]');
  text(xo, yo, txt);

  
  axis equal
  
