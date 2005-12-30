function xyr = align_maps(m1,m2)
% function xyr = align_maps(m1,m2)
%
%

%

nm = size(m1,1);
nm = fix(nm*0.5)*2;

if 0
  e1 = compute_edge(m1(1:2:nm,:), m1(2:2:nm,:));
  e2 = compute_edge(m2(1:2:nm,:), m2(2:2:nm,:));

  a1 = e1(:,3); 
  a2 = e2(:,3);
  sa = e1(:,4) + e2(:,4);
  da = angle_diff(a1, a2);
  [a,saa]  = kalman1(da,sa);
else
  [r1,Srr1,a1,Saa1] = compute_edges(m1);
  [r2,Srr2,a2,Saa2] = compute_edges(m2);
  da = angle_diff(a1,a2);
  Saa = Saa1 + Saa2;
  sa  = diag(Saa);
  [a,saa]  = findMaxAngle(da,Saa);
  saa = saa*saa;
  
end


disp(sprintf('   Angle: %5.2f deg +/- %.3f deg\n' ...
	     ,[da*180/pi, sqrt(sa)*3*180/pi]'));
disp(sprintf('Combined: %5.2f deg +/- %.3f deg\n' ...
	     ,[a*180/pi, sqrt(saa)*3*180/pi]'));

%Align Map 2 with Map1
m2_ = propagate_obs(m2, [0 0 a  0 0 0  0 0 0  0 0 saa]);
%m2_ = translate_obs(m2, [0 0 a]);

dx = m1(:,1) - m2_(:,1);
dy = m1(:,2) - m2_(:,2);

disp(sprintf(' dx,dy: %.2f, %.2f (meters)\n',[dx,dy]'));

mdx = mean(dx); mdy = mean(dy);
disp(sprintf('Translation: %.2f, %.2f (meters)\n',mdx,mdy));

zz = [dx,dy, m1(:,3:6) + m2_(:,3:6)];
xy = kalman2(zz);

disp(sprintf('    Kalman2: %.2f, %.2f (meters)\n',xy(1),xy(2)));


xyr = [xy(1),xy(2), a, xy(3) xy(4) 0  xy(5) xy(6) 0   0 0 saa];

figure
subplot(2,1,1);
hold on
m2_ = propagate_obs(m2,xyr);
mplot(m2_,'bo','b-');
mplot(m1,'ro','r-'); 
m2_ = translate_obs(m2,xyr);
mplot(m2_,'go','g-');

plot_odo(xyr,[],'b+b-b-b-b-',0.1);  
mplot(zz,'b.','b:');

text(m1(:,1), m1(:,2), num2str([1:size(m1,1)]'));
text(m2_(:,1), m2_(:,2), num2str([1:size(m2_,1)]'));

axis equal

if exist('Saa','var')
  subplot(2,1,2);
  xx = linspace(a - 3*sqrt(saa), a + 3*sqrt(saa),1000);
%  xx = linspace(-10*pi, 10*pi,1000);
  inv_Saa = inv(Saa);

  for i = 1:length(xx)
    XX = angle_diff(ones(1, size(Saa,1))*xx(i), da');
    wa(i) = XX*inv_Saa*XX';
  end
  
  XX = ones(1, size(Saa,1))*a - da';
  w_mean = XX*inv_Saa*XX';
  
  [a_max, sigma_a] = findMaxAngle(da,Saa);
  XX = ones(1, size(Saa,1))*a_max - da';
  w_max = XX*inv_Saa*XX';

  [v,i] = min(wa);
  plot(xx*180/pi,exp(-wa),'r.-', ...
       a*180/pi,exp(-w_mean),'bs', ...
       xx(i)*180/pi, exp(-wa(i)),'go', ...
       a_max*180/pi, exp(-w_max), 'mp', ...
       (a_max + 3*sigma_a)*180/pi,0,'b+', ...
       (a_max - 3*sigma_a)*180/pi,0,'b+');
end


function [a_max, sigma_a] = findMaxAngle(a,Saa)
  inv_Saa = inv(Saa);
  A = sum(inv_Saa(:));
  
  B = 0;
  for i = 1:length(a)
    B = B + a(i)*(sum(inv_Saa(:,i)) + sum(inv_Saa(i,:)));
  end
  
  a_max = B/(2*A);
  
  if nargout > 1
    yp = a_max*a_max*A - a_max*B + 1;
    sigma_a = (B + sqrt(B*B + 4*A*yp))/(2*A) - a_max
  end
  

function [x,sxx] = kalman1(z,szz)
n = length(z);

x = z(1); sxx = szz(1);

for i = 2:n
  K = sxx/(sxx + szz(i));
  x = x + K*(angle_diff(z(i), x));
  sxx = (1 - K)*szz(i);
end

function [x,sxx] = kalman2(zz)
n = size(zz,1);

x = zz(1,1:2)'; 
sxx = [zz(1,3:4); zz(1,5:6)];

for i = 2:n
  cov = [zz(i,3:4); zz(i,5:6)];
  K = sxx*inv(sxx + cov);
  x = x + K*(zz(i,1:2)' - x);
  sxx = (eye(2) - K)*cov;
end

if nargout < 2
  x = [x', sxx(1:2), sxx(3:4)];
end

 