function xy = lines2points(l1,l2)
% function lines2points(l1,l2)

global C1 C2
global F1 F2 CX1 CX2

u1 = (CX1 - l1)/F1;
u2 = (CX2 - l2)/F2;

x2 = C2(1)/1000; y2 = C2(2)/1000;

aux1 = tan(atan(u2) + C2(3));
den = 1./(aux1 - u1);
x = (aux1.*x2 - y2).*den;
y = u1.*x;

jb1 = -x.*den;   jb2 = -(1+aux1.*aux1).*(-y2 + x2*u1).*den.*den./(1 + u2.*u2);
jb3 = jb1.*aux1; jb4 = u1.*jb2;

sigma1 = 3*3/F1/F1;
sigma2 = 5*5/F2/F2;

sxx = jb1.*jb1.*sigma1 + jb2.*jb2.*sigma2;
sxy = jb1.*jb3.*sigma1 + jb2.*jb4.*sigma2;
syy = jb3.*jb3.*sigma1 + jb4.*jb4.*sigma2;

xy = [x,y,sxx,sxy,sxy,syy];

return
figure;
a1 = [atan(u1-3*3/F1), atan(u1+3*3/F1)];
a2 = [atan(u2-3*5/F2) + C2(3), atan(u2+3*5/F2) + C2(3)];
R = 10;
padd = zeros(size(a1,1),1);
lx1 = [padd, R*cos(a1),padd];
ly1 = [padd, R*sin(a1),padd];

lx2 = [padd, x2 + R*cos(a2),padd];
ly2 = [padd, y2 + R*sin(a2),padd];
lx2(:,[1,4]) = x2;
ly2(:,[1,4]) = y2;

for i = 1:length(l1);
  cla;
  hold on
  mplot(xy(i,:),'g.','g-');
  line(lx1(i,:)', ly1(i,:)', 'Color','r');
  line(lx2(i,:)', ly2(i,:)','Color','b');
  pause(0.1);
end

