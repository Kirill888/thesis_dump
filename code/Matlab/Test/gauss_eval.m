function p = gauss_eval(x,y,m, cov)
% function p = gauss_eval(x,y,m, cov)
%
%

%
  n = length(x);
  p = zeros(n,1);
  
  inv_cov = inv(cov);

  x = x - m(1);
  y = y - m(2);
  
  p = exp(-0.5*(inv_cov(1,1)*x.*x + x.*y.*(inv_cov(1,2) + inv_cov(2,1)) + ...
                inv_cov(2,2)*y.*y))/(2*pi*sqrt(det(cov)));
  
 
