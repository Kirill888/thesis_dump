function [d,R] = cov_svd(cov)
%function [d,R] = cov_svd(cov)

  eigs = eig(cov);
  
  sxx = eigs(1);
  syy = eigs(2);
  
  d = [sxx 0; 0 syy];
  
  sa2 = (cov(1,1) - sxx)/(syy -sxx);
  
  sa  = sqrt(sa2);
  if sa == 0 
    ca = 1;
  else
    ca  = cov(1,2)/(sa*(sxx - syy));
  end

  R = [ca -sa; 
       sa  ca];
  