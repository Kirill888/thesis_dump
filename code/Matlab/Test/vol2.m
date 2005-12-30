function v = vol2(m1,Cov1, m2, Cov2)
% function v = vol2(m1,Cov1, m2, Cov2)
%
%

  K1 = inv(Cov1);
  dx = m1 - m2;
  n = length(m1);
  B = inv(Cov1 + Cov2);
  
  v =  1/sqrt(det(Cov1 + Cov2))...
      *exp(-0.5*dx'*B*dx) ...
      *1/sqrt((2*pi)^n);


