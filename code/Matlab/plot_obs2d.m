function [h] = plot_obs2d(obs, odo, c)
% function [h] = plot_obs2d(obs, odo,c)
%
%

%

nobs = size(obs,1);
nodo = size(odo,1);

if 1
  [x1,y1] = o2points(obs,1);

  p1 = [x1,y1];
  
  xx1 = zeros(length(x1), nodo);
  yy1 = xx1;
  
  for i = 1:nodo
    pp1 = translate_obs(p1,odo(i,:));
    
    xx1(:,i) = pp1(:,1); yy1(:,i) = pp1(:,2);
  end
  h = plot(xx1,yy1,c);
  
else
  
n = nodo*nobs;

lx1 = zeros(4,n);ly1 = lx1;
lx2 = zeros(5,n); ly2 = lx2;

k = 1;
for i = 1:nobs
  for j = 1:nodo
    [lx1(:,k), ly1(:,k), lx2(:,k), ly2(:,k)] = obs2points(obs(i,:), ...
						  odo(j,:));
    k = k + 1;
  end
end


h1 = line(lx1,ly1); set(h1,'Color',c1);
h2 = line(lx2,ly2); set(h2,'Color',c2);

end

function [lx1,ly1,lx2,ly2] = obs2points(obs,odo)
 da = 1*sqrt(obs(6));
 dr = 1*sqrt(obs(3));
 
 a1 = obs(2) - da + odo(3);
 a2 = obs(2) + da + odo(3);
 r1 = obs(1) - dr;
 r2 = obs(1) + dr;
 
 [xx,yy] = pol2cart([a1;a2;a2;a1], [ r1;r1;r2;r2]);
 
 xx = xx + odo(1);
 yy = yy + odo(2);
 
 lx1 = [odo(1); xx(1); xx(2); odo(1)];
 ly1 = [odo(2); yy(1); yy(2); odo(2)];
 
 lx2 = [xx; xx(1)]; ly2 = [yy; yy(1)];

function [xx, yy] = o2points(obs,sd)
  r = obs(:,1);
  a = obs(:,2);
  dr = sd*sqrt(obs(:,3));
  da = sd*sqrt(obs(:,6));
  
  a1 = a - da;
  a2 = a + da;
  r1 = r + dr;
  r2 = r - dr;

  x1 = r1.*cos(a1); y1 = r1.*sin(a1);
  x2 = r1.*cos(a2); y2 = r1.*sin(a2);
  x3 = r2.*cos(a2); y3 = r2.*sin(a2);
  x4 = r2.*cos(a1); y4 = r2.*sin(a1);
  
  xx = zeros(length(x1)*6 + 1,1);
  yy = xx;
  
  xx(1:6:end) = 0  ; yy(1:6:end) =  0 ;
  xx(2:6:end) = x1'; yy(2:6:end) = y1';
  xx(3:6:end) = x2'; yy(3:6:end) = y2';
  xx(4:6:end) = x3'; yy(4:6:end) = y3';
  xx(5:6:end) = x4'; yy(5:6:end) = y4';
  xx(6:6:end) = x3'; yy(6:6:end) = y3';
