function e = compute_edges(m)
% function e = compute_edges(m)
%
%
%  e = [r,Srr,l1,l2,x,y,a]

  n = size(m,1);
  ne = n*(n-1);
  
  if ne <= 0
    e = zeros(0,7);
    return
  end
  
  from = zeros(ne,1);
  to   = zeros(ne,1);
  
  W = 1:(n-1);
  for i = 1:n
    from(W+(i-1)*(n-1)) = ones(n-1,1)*i;
      to(W+(i-1)*(n-1)) = setdiff(1:n,i);
  end
  
  
  %Compute length
  
  dx = m(to,1) - m(from,1);
  dy = m(to,2) - m(from,2);
  dx2 = dx.*dx;  dy2 = dy.*dy; dxdy = dx.*dy;
  
  r2 = dx2 + dy2;
  r  = sqrt(r2);
  
  sxx = m(from,3) + m(to,3);
  sxy = m(from,4) + m(to,4);
  syy = m(from,6) + m(to,6);
  
  Srr = dx2.*sxx + dy2.*syy + 2*dxdy.*sxy./r2;
  
  ii = find(Srr < 0);
  Srr(ii) = nan;
  
  x = 0.5*(m(to,1) + m(from,1));
  y = 0.5*(m(to,2) + m(from,2));
  a = atan2(dy,dx);

  e = [r,Srr,from,to,x,y,a];




