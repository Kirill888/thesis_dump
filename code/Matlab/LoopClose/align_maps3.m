function [xyr,x_opt,xyr0] = align_maps3(m1,m2)
%  function [xyr,x_opt,xyr0] = align_maps3(m1,m2)
%
%
%

n = size(m1,1);

[xya1,cov1] = compute_angles(m1);
[xya2,cov2] = compute_angles(m2);

disp(sprintf('a1 = %.2f (deg)\n',xya1(3:end)*180/pi));
disp(sprintf('a2 = %.2f (deg)\n',xya2(3:end)*180/pi));

obs = zeros(size(xya1));

obs(1:2)   = xya1(1:2) - xya2(1:2);
obs(3:end) = angle_diff(xya1(3:end), xya2(3:end));



cov = cov1 + cov2;

disp(sprintf('obs = %.2f (deg)\n',obs(3:end)*180/pi));

M = inv(cov);
C = obs;

A = zeros(3);

A(1,1) = M(1,1);
A(2,2) = M(2,2);
A(3,3) = sum(sum(M(3:end,3:end)));

A(1,2) = 0.5*(M(2,1) + M(1,2));
A(1,3) = 0.5*(sum(M(3:end,1)) + sum(M(1,3:end)));
A(2,3) = 0.5*(sum(M(3:end,2)) + sum(M(2,3:end)));

A(2,1) = A(1,2);
A(3,1) = A(1,3);
A(3,2) = A(2,3);

b = zeros(3,1);

b(1) = C*M(1,:)' + C*M(:,1);
b(2) = C*M(2,:)' + C*M(:,2);
b(3) = sum( C*M(3:end,:)' + C*M(:,3:end) );

xy_cov = inv(A);
xyr = 0.5*xy_cov*b

%xyr0 -- location of the mean of map2 in map1
xyr0 = [xyr(1:2)' + xya2(1:2), xyr(3), xy_cov(1:3), xy_cov(4:6), xy_cov(7:9)];

%xyr  -- location of the coordinate frame of map2 in map1
xyr = propagate_odo([-xya2(1:2), zeros(1,10)], xyr0);

%Do optimal search

[x_opt,fv,exflag,output] = fminsearch(@optimise_eval, xyr(1:3), [], ...
				      m1,m2);
disp(sprintf('Optimisation completed in %d steps',output.iterations));

% Do plotting

figure
mplot(m1,'r.','r-');hold on
axis equal
mplot(m2,'g.','g-');
text(m1(:,1),m1(:,2),num2str([1:n]'));
text(m2(:,1),m2(:,2),num2str([1:n]'));

plot(xya1(1),xya1(2),'ro',xya2(1),xya2(2),'go');


m2_ = translate_obs(m2,  [-xya2(1:2),   0]);
m2_ = translate_obs(m2_, [   0, 0   , xyr(3)]);
m2_ = translate_obs(m2_, [xya1(1:2) ,   0]);
text(m2_(:,1),m2_(:,2),num2str([1:n]'));

mplot(m2_,'mo','m-');

mplot(propagate_obs(m2,xyr),'bs','b-');
plot_odo(xyr0,[],'coc-c-c-c-c-c-',1);
plot_odo(xyr,[],'bs-b-b-b-b-b-b-',1);
plot_robot(x_opt,'m');


function w = optimise_eval(x,m1,m2)
  w = map_overlap(m1,m2,x);
  w = -sum(w,2);
 

