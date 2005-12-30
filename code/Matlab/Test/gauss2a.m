function a = gauss2a(m,S)
% Convert
%    1/(2*pi*sqrt(|S|))*exp(-0.5*((x-m)'*inv(S)*(x-m)))
% in to equivalent 
%  exp(a(1)*x.*x + a(2)*y.*y + a(3)*x.*y + a(4)*x + a(5)*y + a(6))

if nargin == 1
  S = [m(3:4);m(5:6)];
  m = m(1:2);
end

  if 1
    
  sxx = S(1,1);
  syy = S(2,2);
  sxy = S(1,2);
  mx  = m(1);
  my  = m(2);
  
  a(1) = -syy;
  a(2) = -sxx;
  a(3) = 2*sxy;
  a(4) = +2*syy*mx - 2*sxy*my;
  a(5) = -2*sxy*mx + 2*sxx*my;
  a(6) = -sxx*my^2 - syy*mx^2 + 2*sxy*my*mx;

  a = -0.5*a/(-sxx*syy+sxy^2);
  
  a(6) = a(6) - log(sqrt(det(2*pi*S)));
 
  else
  
  sxx = S(1,1);
  syy = S(2,2);
  sxy = S(1,2);
  sx  = sqrt(sxx);
  sy  = sqrt(syy);
  sxsy = sx*sy;
  p = sxy/sxsy;
  
  mx  = m(1);
  my  = m(2);
  
  a(1) = 1/sxx;
  a(2) = 1/syy;
  a(3) = -2*p/sxsy;
  a(4) = -2*mx/sxx + 2*p*my/sxsy;
  a(5) = -2*my/syy + 2*p*mx/sxsy;
  a(6) = mx*mx/sxx + my*my/syy - 2*p*mx*my/sxsy;
  
  a = a/(-2*(1-p*p));
  a(6) = a(6) - log(2*pi*sxsy*sqrt(1 - p*p));
  
  end
  
