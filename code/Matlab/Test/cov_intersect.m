function [x,cov, Norm] = cov_intersect(a,Paa, b, Pbb, w)
  
  if size(a,1) == 1
    a = a';
  end
  
  if size(b,1) == 1
    b = b';
  end
  
  Paa_inv = inv(Paa);
  Pbb_inv = inv(Pbb);
  
  n = length(w);
  cov = zeros(2,2,n);
  x   = zeros(n,2);
  if nargout > 2
    Norm = zeros(n,2);
  end
  
  
  for i = 1:n
    Pcc_inv = w(i)*Paa_inv + (1-w(i))*Pbb_inv;
    cov(:,:,i) = inv(Pcc_inv);
    
    x(i,:) = (cov(:,:,i)*(w(i)*Paa_inv*a + (1-w(i))*Pbb_inv*b))';
    
    if nargout > 2
      e = eigs(cov(:,:,i));
      Norm(i,1) = norm(cov(:,:,i));
      Norm(i,2) = sqrt(e'*e);
    end
    
  end