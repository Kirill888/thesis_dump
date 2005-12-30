function v = gauss_d2(x,  y, a)
%function v = gauss_d2(x, y, a)

%  exp(a(1)*x.*x + a(2)*y.*y + a(3)*x.*y + a(4)*x + a(5)*y)*a(6)



  nx = length(x);
  ny = length(y);

  v = zeros(ny,nx);
  
  if size(x,1) > size(x,2)
    x = x';
  end
  
  for j = 1:ny
    v(j,:) = a(6)*exp(a(1)*x.*x + a(2)*y(j).*y(j) + a(3)*x.*y(j) + a(4)*x ...
		      + a(5)*y(j));
  end
  
  

