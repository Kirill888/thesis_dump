function aa = tst3

x = -10:0.1:10;
y = -11:0.1:11;

m1 = [1 3];
S1 = [1 0; 0 4];
S1 = rot(pi/4)*S1*rot(pi/4)';


m2 = [1 3];
S2 = [1 0; 0 9];
S2 = rot(-pi/4)*S1*rot(-pi/4)';

a1 = gauss2a(m1,S1);
a2 = gauss2a(m2,S2);

aa = a1(1:5) + a2(1:5);
aa(6) = a1(6)*a2(6);

v1 = gauss_d2(x,y,a1);
v2 = gauss_d2(x,y,a2);

vv = gauss_d2(x,y,aa);

%vv_ = gauss_d(x,y,m1,S1).*gauss_d(x,y,m2,S2);
%sum(sum(vv-vv_))

figure

subplot(1,2,1);
mesh(x,y,v1); hold on; mesh(x,y,v2);


subplot(1,2,2);
pcolor(x,y,vv);
axis equal

dblquad('gauss_d2',-100,100,-100,100,[],[],aa)
volume(aa)

function R = rot(a);
  R = [cos(a) sin(a); -sin(a) cos(a)];

