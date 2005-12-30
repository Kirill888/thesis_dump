function tst_vol(m1,Cov1,m2,Cov2)
% function tst_vol(m1,Cov1,m2,Cov2)
%
%

figure;

[min1,max1] = gauss_bounds(m1,Cov1);
[min2,max2] = gauss_bounds(m2,Cov2);

x0 = min(min1(1),min2(1));
y0 = min(min1(2),min2(2));

x1 = max(max1(1),max2(1));
y1 = max(max1(2),max2(2));

nx = 60; ny = 50;
M1 = zeros(ny,nx);
M2 = zeros(ny,nx);

xx = linspace(x0,x1,nx);
yy = linspace(y0,y1,ny);

for i = 1:nx
  x = ones(ny,1)*xx(i);
  M1(:,i) = gauss_eval(x,yy',m1,Cov1);
  M2(:,i) = gauss_eval(x,yy',m2,Cov2);
end

M = M1.*M2;

subplot(2,2,1); pcolor(xx,yy,M1);
subplot(2,2,2); pcolor(xx,yy,M2);
subplot(2,2,3); pcolor(xx,yy,M);

dx = xx(2) - xx(1);
dy = yy(2) - yy(1);

sum(sum(M))*dx*dy - vol2(m1,Cov1,m2,Cov2)

function [p_min,p_max] = gauss_bounds(m,cov)

dx = sqrt(cov(1,1))*3;
dy = sqrt(cov(2,2))*3;

p_min = [m(1) - dx, m(2) - dy];
p_max = [m(1) + dx, m(2) + dy];


  

function w = dblquad_eval(x,y,  m1,cov1, m2, cov2)
  p1 = gauss_eval(x,y,m1,cov1);
  p2 = gauss_eval(x,y,m2,cov2);
  w = p1*p2;
