function [x,w] = sample_gauss(rmean,rcov,NP)
%function [x,w] = sample_gauss(rmean,rcov,NP)
%
%
%

  if isempty(rcov)
    rcov = [rmean(4:6); rmean(7:9); rmean(10:12)];
  end
  
  
  l = chol(rcov);

  x = randn(NP,3);
  n = size(rcov,1);

  x = x*l;

  w = 1./sqrt((2*pi)^n*det(rcov))*exp(-0.5*sum(x*inv(rcov).*x,2));

  
  x(:,1) = x(:,1) + rmean(1);
  x(:,2) = x(:,2) + rmean(2);
  x(:,3) = x(:,3) + rmean(3);
