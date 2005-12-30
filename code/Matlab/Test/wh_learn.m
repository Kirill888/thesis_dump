function [w,b] = wh_learn(x_pos, x_neg)

  xx = [x_pos(:,1:2); x_neg(:,1:2)];
  yy = [ones(size(x_pos,1),1); -ones(size(x_neg,1),1)];

  nx = size(xx,1);
  %Solve the optimum solution
  X = [xx, ones(nx,1)];
  
  W = inv(X'*X)*X'*yy;
  w = W(1:2)
  b = W(3)
  
  figure
  plot(x_pos(:,1),x_pos(:,2),'ro', ...
       x_neg(:,1),x_neg(:,2),'bs');

  xl = [min(xx(:,1)); max(xx(:,1))];
  yl = -(b + w(1).*xl)/w(2);
  hold on
  line(xl,yl);
  axis equal
  
  