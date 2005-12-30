function res = gauss_line_intersect(M,cov, l)


A = inv(cov);
m = l(1);
b = l(2);

M0 = M;

M = [M(1), M(2) - b];

v = [1,m];

a = v*A*v';
b = -(v*A*M' + M*A*v');
c = M*A*M';

scaler = 1/(2*pi*sqrt(det(cov)));

res = [a,b,c,scaler];

figure(1);
clf
subplot(2,1,1);
mplot([M0, cov(1,:), cov(2,:)],'ro','r-');
axis equal
ax = axis;
x = linspace(ax(1),ax(2),100);
y = x*l(1) + l(2);
hold on;
plot(x,y,'b-');

subplot(2,1,2);
y = scaler*exp(-0.5*(a*x.*x + b*x + c));
plot(x,y,'b.-');

w_numeric = trapz(x,y);

w = sqrt(2*pi/a)*exp(b*b/(8*a) - 0.5*c)*scaler;
w_log = -1/2*(log(2*pi) + log(a*det(cov)) - b*b/(4*a) + c);

title(sprintf('Integral: %g (%g) [%g, %g]',w,w_numeric, w_log, log(w)));
