function v = gauss_d(x,  y, m, Pmm)
%function v = gauss_d(x, y,  m, Pmm)
  m1 = m(1);
  m2 = m(2);
  
  s11 = Pmm(1,1);
  s12 = Pmm(1,2);
  s22 = Pmm(2,2);
  
  s1 = sqrt(s11);
  s2 = sqrt(s22);
  
  p  = s12/(s1*s2);
  
  x1 = x - m1;
  x2 = y - m2;

  nx = length(x);
  ny = length(y);

  v = zeros(ny,nx);
  
  if size(x1,1) > size(x1,2)
    x1 = x1';
  end
  
  for j = 1:ny
    z = x1.*x1/s11 - 2*p*x1*x2(j)/(s1*s2) + x2(j)*x2(j)/s22;
    v(j,:) = 1/(2*pi*s1*s2*sqrt(1-p*p))*exp(-z*0.5/(1-p*p));
  end
  
  

